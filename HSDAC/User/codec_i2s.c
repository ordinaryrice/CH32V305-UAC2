#include "codec_i2s.h"

#include <string.h>
#include <stdatomic.h>

#include "ch32v30x_gpio.h"
#include "ch32v30x_rcc.h"
#include "ch32v30x_spi.h"
#include "ch32v30x_dma.h"
#include "ch32v30x_exti.h"
#include "ch32v30x_tim.h"
#include "ch32v30x_misc.h"
#include "audio_dsp.h"
#include "config.h"

// --------------------------------------------------------------------------------
// marco
// --------------------------------------------------------------------------------

#define I2S_DMA_BUFFER_SIZE_MASK (I2S_DMA_BUFFER_SIZE - 1)

// --------------------------------------------------------------------------------
// type
// --------------------------------------------------------------------------------

// --------------------------------------------------------------------------------
// variable
// --------------------------------------------------------------------------------

static struct StereoSample i2s_dma_buffer_[I2S_DMA_BUFFER_SIZE] = {0};
static uint32_t last_i2s_dma_wpos_ = 0;
static volatile uint8_t i2s_lrck_sync_stage_ = 0;
static volatile uint8_t i2s_sync_lrck = 0;
static volatile int8_t bitshift = 32;
static uint32_t carry_over = 0;
static volatile uint16_t bck_last_count_read_ = 0;
static volatile uint32_t bck_last_valid_khz_ = 0;
static uint8_t bck_measure_baseline_valid_ = 0;
static volatile uint32_t zero_fill_samples_ = 0;
static volatile uint32_t dma_min_free_samples_ = I2S_DMA_BUFFER_SIZE;
static volatile uint32_t dma_min_samples_have_ = I2S_DMA_BUFFER_SIZE;
static volatile uint32_t dma_max_samples_have_ = 0;
static uint32_t dma_samples_have_ = 0;
static volatile uint32_t dma_underflow_samples_ = 0;

// --------------------------------------------------------------------------------
// declare
// --------------------------------------------------------------------------------
static uint8_t SyncToLrck(void);
static void Start_Sync(void);
static void Stop_Sync(void);

__attribute__((interrupt("WCH-Interrupt-fast"), used))
void EXTI15_10_IRQHandler(void);

// --------------------------------------------------------------------------------
// implement
// --------------------------------------------------------------------------------

void CodecI2s_BckMeasure_Init(void) {
    /*
     * TIM3 counts BCK edges directly on PC6 via its external clock input.
     * USB reads the delta from the counter on the SOF cadence; the timer
     * itself remains active continuously in the background.
     */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &gpio);

    TIM_DeInit(TIM3);
    TIM_TimeBaseInitTypeDef tim_base = {0};
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base.TIM_Period = 0xFFFFu;
    tim_base.TIM_Prescaler = 0u;
    TIM_TimeBaseInit(TIM3, &tim_base);

    TIM_TIxExternalClockConfig(TIM3,
                              TIM_TIxExternalCLK1Source_TI1,
                              TIM_ICPolarity_Rising,
                              0u);

    TIM3->CNT = 0u;
    bck_last_count_read_ = 0u;
    bck_last_valid_khz_ = 0u;
    bck_measure_baseline_valid_ = 0u;
    TIM_Cmd(TIM3, ENABLE);
}

uint32_t CodecI2s_GetBckKHzApprox(void) {
    uint16_t now = (uint16_t)TIM3->CNT;
    if (!bck_measure_baseline_valid_) {
        bck_last_count_read_ = now;
        bck_measure_baseline_valid_ = 1u;
        return 0u;
    }

    uint16_t delta = (uint16_t)(now - bck_last_count_read_);
    bck_last_count_read_ = now;

    if (delta == 0u) {
        return bck_last_valid_khz_;
    }

    /*
     * The timer counts rising edges, so each count represents one BCK period.
     * SOF sampling occurs every 1 ms, so the edge count is the frequency in kHz.
     */
    bck_last_valid_khz_ = delta;
    return bck_last_valid_khz_;
}

static void DMA_Tx_Init(DMA_Channel_TypeDef* DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize) {
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA_CHx);

    DMA_InitTypeDef DMA_InitStructure={0};
    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);
}

static void I2S2_Init(void) {
    /*
     * USB -> I2S bridge is configured as a generic I2S Slave TX.
     * BCLK/WS are driven by the external I2S master, while CH32V305 only
     * transmits audio data on the SD line.
     */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    SPI_I2S_DeInit(SPI2);

    GPIO_InitTypeDef GPIO_InitStructure={0};

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_SET);

    /* BCLK (PB13) and WS (PB12) are inputs in slave mode. */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    /* SD (PB15) is the data output pin in generic I2S. */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /*
     * PC7 selects the external clock family:
     *  - LOW  => 22.5792MHz family for 44.1kHz-based rates
     *  - HIGH => 24.576MHz family for 48kHz-based rates
     *
     * PC8 and PC9 are then used as the divider select lines within each family:
     *  - PC8: x4 when HIGH, x1 when LOW
     *  - PC9: x2 when HIGH, x1 when LOW (applied after the PC8 choice)
     * This selects the overall input divider ratio as x1/x2/x4/x8.
     */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_RESET);
    GPIO_WriteBit(GPIOC, GPIO_Pin_7, Bit_RESET);
    GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);

    I2S_InitTypeDef I2S_InitStructure={0};
    /*
     * Normal I2S (Philips standard): LRCK transitions one BCK before the first
     * bit of each sample. The external clock is assumed to be 64 BCK per LRCK,
     * so the CH32V305 remains in 32-bit stereo mode and emits one 32-bit word
     * per channel per LRCK period.
     */
    I2S_InitStructure.I2S_Mode = I2S_Mode_SlaveTx;
    I2S_InitStructure.I2S_Standard = I2S_Standard_Phillips;
    I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_32b;
    I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
    I2S_InitStructure.I2S_AudioFreq = I2S_AudioFreq_Default;
    I2S_InitStructure.I2S_CPOL = I2S_CPOL_Low;
    I2S_Init(SPI2, &I2S_InitStructure);

    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
    // I2S_Cmd(SPI2, ENABLE);
}

static int32_t Swap16(int32_t x) {
    uint32_t r = x;
    uint32_t up = r & 0xffff;
    uint32_t down = r >> 16;
    return down | (up << 16);
}

static uint32_t GetDmaReadPosInternal() {
    uint32_t a = (I2S_DMA_BUFFER_SIZE * 4 - DMA_GetCurrDataCounter(DMA1_Channel5)) / 4;
    return a & I2S_DMA_BUFFER_SIZE_MASK;
}

static uint32_t GetDmaSamplesHaveInternal() {
    uint32_t read_pos = GetDmaReadPosInternal();
    uint32_t prev_have = dma_samples_have_;
    dma_samples_have_ = (last_i2s_dma_wpos_ - read_pos) & I2S_DMA_BUFFER_SIZE_MASK;

    /*
     * Underflow recovery: if have >= I2S_DMA_BUFFER_SIZE - 1 the write
     * pointer has fallen behind the DMA read pointer (wpos <= rpos in the
     * modular sense).  In this state GetDmaCanWrite() returns only 1 or 2
     * samples, starving the ring indefinitely.
     *
     * Recover by snapping last_i2s_dma_wpos_ to read_pos so that the ring
     * appears empty (have=0) and GetDmaCanWrite() returns the full capacity.
     * This discards the position fiction and lets the write side catch up.
     * A small amount of stale data may be output until audio refills the
     * ring, but this is better than permanent starvation.
     *
     * Threshold: have >= 250 (6 samples below full) to avoid triggering on
     * the normal pre-buffer cushion (have=245 at startup).
     */
    if (dma_samples_have_ >= I2S_DMA_BUFFER_SIZE - 6u) {
        dma_underflow_samples_ += dma_samples_have_;
        /*
         * Snap the write pointer ahead of the read pointer by half the ring
         * size.  Snapping to exactly read_pos (have=0) leaves no cushion and
         * the very next DMA advance will cause another underflow immediately.
         * Half-ring ahead means the DMA must consume 128 more samples before
         * it catches the write pointer again, giving ~2.67 ms at 48 kHz for
         * the FIFO to refill the ring before the next underflow.
         *
         * The buffer region between read_pos and the new wpos contains stale
         * or zero data so a brief glitch is unavoidable, but it is far shorter
         * and less frequent than repeated back-to-back underflow snaps.
         */
        last_i2s_dma_wpos_ = (read_pos + I2S_DMA_BUFFER_SIZE / 2) & I2S_DMA_BUFFER_SIZE_MASK;
        dma_samples_have_ = I2S_DMA_BUFFER_SIZE / 2;
    } else {
        /* Accumulate smaller jumps as underflow evidence */
        if (dma_samples_have_ > prev_have) {
            uint32_t jump = dma_samples_have_ - prev_have;
            if (jump > UAC_MAX_PACKAGE_SIZE) {
                dma_underflow_samples_ += jump;
            }
        }
    }

    if (dma_samples_have_ < dma_min_samples_have_) dma_min_samples_have_ = dma_samples_have_;
    if (dma_samples_have_ > dma_max_samples_have_) dma_max_samples_have_ = dma_samples_have_;
    return dma_samples_have_;
}

static uint32_t GetDmaCanWrite() {
    uint32_t have = GetDmaSamplesHaveInternal();
    if (have >= I2S_DMA_BUFFER_SIZE) return 0;
    uint32_t free_samples = I2S_DMA_BUFFER_SIZE - have;
    return free_samples > (I2S_DMA_BUFFER_SIZE - 1) ? (I2S_DMA_BUFFER_SIZE - 1) : free_samples;
}

static void ResetDmaBufferState(void) {
    memset(i2s_dma_buffer_, 0, sizeof(i2s_dma_buffer_));
    last_i2s_dma_wpos_ = 0;
    dma_samples_have_ = 0;
    dma_min_samples_have_ = I2S_DMA_BUFFER_SIZE;
    dma_max_samples_have_ = 0;
    dma_min_free_samples_ = I2S_DMA_BUFFER_SIZE;
    dma_underflow_samples_ = 0;
    /*
     * carry_over holds the right-channel word from the previous stereo sample
     * and is used by StoreStereoSamples() when bitshift != 0 (e.g. 48 kHz).
     * Resetting it here forces the first sample after every restart to emit
     * zeros in the lower 16 bits, producing a one-sample glitch at each
     * DMA/I2S restart.  Leave it unchanged so the shift chain is continuous
     * across soft restarts.  carry_over is only zeroed at hard init time
     * (CodecI2s_Init) where a glitch is acceptable.
     */
}

static void RestartDmaTxFromBoundary(void) {
    const uint16_t dma_words = (uint16_t)(I2S_DMA_BUFFER_SIZE * sizeof(struct StereoSample) / sizeof(uint16_t));

    /* Stop DMA first */
    DMA_Cmd(DMA1_Channel5, DISABLE);
    DMA_ClearFlag(DMA1_FLAG_GL5 | DMA1_FLAG_TC5 | DMA1_FLAG_HT5 | DMA1_FLAG_TE5);
    DMA_SetCurrDataCounter(DMA1_Channel5, dma_words);
    DMA1_Channel5->MADDR = (uint32_t)&i2s_dma_buffer_[0];

    /*
     * Disable I2S to reset its internal shift register and channel counter.
     * While I2S is enabled but DMA is stopped, the peripheral keeps clocking
     * internally and its channel position drifts unpredictably.  Disabling
     * I2S here guarantees a clean internal state before re-synchronisation.
     */
    I2S_Cmd(SPI2, DISABLE);

    /*
     * Re-enable I2S *before* enabling DMA.
     * In slave mode the peripheral waits for the next WS edge after I2SE is
     * set, so it will self-align to the LRCK boundary.  DMA is then enabled
     * immediately so the first word is ready in the TX FIFO before the
     * peripheral actually starts shifting out data.
     */
    i2s_sync_lrck = 1;
    Start_Sync();
}

void Start_Sync(void)
{
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
    EXTI_ClearITPendingBit(EXTI_Line12);
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    EXTI_InitStructure.EXTI_Line = EXTI_Line12;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

void Stop_Sync(void)
{
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);
}

__attribute__((interrupt("WCH-Interrupt-fast"), used))
void EXTI15_10_IRQHandler(void) {
    if (i2s_sync_lrck == 1) {
        DMA_Cmd(DMA1_Channel5, ENABLE);
        i2s_sync_lrck = 2;
    }
    else if (i2s_sync_lrck == 2) {
        SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
        i2s_sync_lrck = 3;
    }
    else if (i2s_sync_lrck == 3) {
        I2S_Cmd(SPI2, ENABLE);
        i2s_sync_lrck = 0;
        Stop_Sync();
    }
    EXTI_ClearITPendingBit(EXTI_Line12);
}

static void StoreStereoSamples(struct StereoSample* src, uint32_t count, uint32_t pos) {
    uint32_t left_shifted, right_shifted;
    uint16_t* dst = (uint16_t*)&i2s_dma_buffer_[pos];
    for (uint32_t i = 0; i < count; ++i) {
        volatile uint32_t left = (uint32_t)src[i].left;
        volatile uint32_t right = (uint32_t)src[i].right;
        // left  = 0x3C6EF372u;
        // right = 0xFE94F82Au;
        // left  = 0x80000000u;
        // right = 0x00000000u;

        /* Keep the established 16-bit sample alignment when packing stereo I2S words. */
        if (bitshift) {
            left_shifted = (left << bitshift) | (carry_over >> (32u - bitshift));
            right_shifted = (right << bitshift) | (left >> (32u - bitshift));
        } else {
            left_shifted = left;
            right_shifted = right;
        }

        dst[0] = (uint16_t)(left_shifted >> 16);
        dst[1] = (uint16_t)left_shifted;
        dst[2] = (uint16_t)(right_shifted >> 16);
        dst[3] = (uint16_t)right_shifted;
        carry_over = right;
        dst += 4;
    }
}

static void I2S2_PrescaleConfig(uint32_t v) {
    (void)v;
}

static void SetExternalClockDiv(uint32_t sample_rate) {
    /*
     * The external clock device is selected by PC7:
     *  - LOW  => 22.5792MHz family (44.1kHz-based rates)
     *  - HIGH => 24.576MHz family (48kHz-based rates)
     *
     * Within each family, PC8/PC9 provide the divider ratio as:
     *  effective ratio = (PC8 ? 4 : 1) * (PC9 ? 2 : 1)
     *  => x1, x2, x4, x8
     *
     * For 32-bit stereo I2S, BCK = fs * 64.
     * 22.5792MHz / 8 = 2.8224MHz -> 44.1kHz
     * 22.5792MHz / 4 = 5.6448MHz -> 88.2kHz
     * 22.5792MHz / 2 = 11.2896MHz -> 176.4kHz
     * 22.5792MHz / 1 = 22.5792MHz -> 352.8kHz
     *
     * 24.576MHz / 8 = 3.072MHz -> 48kHz
     * 24.576MHz / 4 = 6.144MHz -> 96kHz
     * 24.576MHz / 2 = 12.288MHz -> 192kHz
     */
    uint8_t pc7 = Bit_RESET;      // clock generator selection: 1=22.5792MHz, 0=24.576MHz
    uint8_t pc8 = Bit_RESET;    // clock divider selection: 0=x1, 1=x4
    uint8_t pc9 = Bit_RESET;    // clock divider selection: 0=x1, 1=x2

    switch (sample_rate) {
        case 384000:
            pc7 = Bit_RESET;
            bitshift = 0;
            break;
        case 192000:
            pc7 = Bit_RESET;
            pc9 = Bit_SET;
            bitshift = 0;
            break;
        case 96000:
            pc7 = Bit_RESET;
            pc8 = Bit_SET;
            bitshift = 0;
            break;
        case 48000:
            pc7 = Bit_RESET;
            pc8 = Bit_SET;
            pc9 = Bit_SET;
            bitshift = 16;
            break;

        case 352800:
            pc7 = Bit_SET;
            bitshift = 0;
            break;
        case 176400:
            pc7 = Bit_SET;
            pc9 = Bit_SET;
            bitshift = 0;
            break;
        case 88200:
            pc7 = Bit_SET;
            pc8 = Bit_SET;
            bitshift = 0;
            break;
        case 44100:
        default:
            pc7 = Bit_SET;
            pc8 = Bit_SET;
            pc9 = Bit_SET;
            bitshift = 0;
            break;
    }

    GPIO_WriteBit(GPIOC, GPIO_Pin_7, pc7);
    GPIO_WriteBit(GPIOC, GPIO_Pin_8, pc8);
    GPIO_WriteBit(GPIOC, GPIO_Pin_9, pc9);
}

// --------------------------------------------------------------------------------
// public
// --------------------------------------------------------------------------------

void CodecI2s_Init(void) {
    carry_over = 0;
    CodecI2s_BckMeasure_Init();
    I2S2_Init();
    DMA_Tx_Init(DMA1_Channel5, (u32)&SPI2->DATAR, (u32)&i2s_dma_buffer_, I2S_DMA_BUFFER_SIZE * sizeof(struct StereoSample) / sizeof(uint16_t));
    CodecI2s_SetSampleRate(48000);
    RestartDmaTxFromBoundary(); // 位相合わせのため必須
}

void CodecI2s_DeInit(void) {
    DMA_Cmd(DMA1_Channel5, DISABLE);
    DMA_DeInit(DMA1_Channel5);
    I2S_Cmd(SPI2, DISABLE);
    SPI_I2S_DeInit(SPI2);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, DISABLE);
}

void CodecI2s_Start(void) {
    ResetDmaBufferState();
    /*
     * Pre-fill the write pointer to the midpoint of the ring before the DMA
     * starts reading from position 0.  This creates a ~2.67 ms cushion
     * (128 samples at 48 kHz) between the DMA read head and the software
     * write head so that brief gaps in USB supply do not immediately cause
     * a DMA underrun.
     *
     * The buffer is already zeroed by ResetDmaBufferState(), so the DMA will
     * output silence during the pre-fill region until real audio catches up.
     */
    last_i2s_dma_wpos_ = I2S_DMA_BUFFER_SIZE / 2;
    RestartDmaTxFromBoundary();
}

void CodecI2s_Stop(void) {
    DMA_Cmd(DMA1_Channel5, DISABLE);
    DMA_ClearFlag(DMA1_FLAG_GL5 | DMA1_FLAG_TC5 | DMA1_FLAG_HT5 | DMA1_FLAG_TE5);
    I2S_Cmd(SPI2, DISABLE);
    SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_OVR | SPI_I2S_FLAG_BSY | SPI_I2S_FLAG_TXE);
    ResetDmaBufferState();
}

uint32_t CodecI2s_WriteUACBuffer(struct StereoSample* src, uint32_t len) {
    uint32_t can_write = GetDmaCanWrite();
    can_write = len < can_write ? len : can_write;

    uint32_t fill1 = I2S_DMA_BUFFER_SIZE - last_i2s_dma_wpos_;
    fill1 = fill1 < can_write ? fill1 : can_write;
    uint32_t fill2 = can_write - fill1;

    StoreStereoSamples(src, fill1, last_i2s_dma_wpos_);
    if (fill2 != 0) {
        StoreStereoSamples(src + fill1, fill2, 0);
    }

    last_i2s_dma_wpos_ += can_write;
    last_i2s_dma_wpos_ &= I2S_DMA_BUFFER_SIZE_MASK;

    return can_write;
}

void CodecI2s_FillZero(uint32_t len) {
    if (len == 0) return;

    zero_fill_samples_ += len;

    uint32_t fill1 = I2S_DMA_BUFFER_SIZE - last_i2s_dma_wpos_;
    fill1 = fill1 < len ? fill1 : len;
    uint32_t fill2 = len - fill1;

    struct StereoSample* dst = &i2s_dma_buffer_[last_i2s_dma_wpos_];
    while (fill1--) {
        dst->left = 0;
        dst->right = 0;
        ++dst;
    }

    dst = &i2s_dma_buffer_[0];
    while (fill2--) {
        dst->left = 0;
        dst->right = 0;
        ++dst;
    }

    last_i2s_dma_wpos_ += len;
    last_i2s_dma_wpos_ &= I2S_DMA_BUFFER_SIZE_MASK;
}

void CodecI2s_FillZeroIfTooSmallData() {
    /*
     * Artificial bulk filling is disabled. It treats a low application FIFO as
     * a DMA underrun and can overwrite valid ring-buffer positions with zeros
     * or repeated samples. Compensation belongs only to a measured underrun,
     * not to this FIFO-size/DMA-size comparison.
     */
}

uint32_t CodecI2s_GetFreeSpace() {
    uint32_t free_space = GetDmaCanWrite();
    if (free_space < dma_min_free_samples_) {
        dma_min_free_samples_ = free_space;
    }
    return free_space;
}

uint32_t CodecI2s_GetZeroFillSamples(void) {
    return zero_fill_samples_;
}

uint32_t CodecI2s_GetDmaReadPos(void) {
    return GetDmaReadPosInternal();
}

uint32_t CodecI2s_GetDmaWritePos(void) {
    return last_i2s_dma_wpos_;
}

uint32_t CodecI2s_GetDmaSamplesHave(void) {
    return GetDmaSamplesHaveInternal();
}

uint32_t CodecI2s_GetDmaMinSamplesHave(void) {
    return dma_min_samples_have_;
}

uint32_t CodecI2s_GetDmaMaxSamplesHave(void) {
    return dma_max_samples_have_;
}

void CodecI2s_ResetIntervalStats(void) {
    /* Reset per-interval min/max so the next log period reflects only the
     * samples observed since the last reset, not the all-time extremes. */
    dma_min_samples_have_ = dma_samples_have_;
    dma_max_samples_have_ = dma_samples_have_;
    dma_min_free_samples_ = I2S_DMA_BUFFER_SIZE;
}

uint32_t CodecI2s_GetDmaMinFreeSamples(void) {
    return dma_min_free_samples_;
}

uint32_t CodecI2s_GetDmaUnderflowSamples(void) {
    return dma_underflow_samples_;
}

void CodecI2s_SetSampleRate(uint32_t sample_rate) {
    /*
     * The external 24.576MHz oscillator is divided by the PC6/PC7 selector pins,
     * so the sample-rate metadata is translated to the required divider ratio.
     * The DMA stream is stopped while changing the divider to avoid a frame
     * boundary shift between the audio clock selection and the active I2S DMA.
     */
    CodecI2s_Stop();
    SetExternalClockDiv(sample_rate);
    CodecI2s_Start();
}

int32_t CodecI2s_GetCurrentSizeDiffFromCenter(uint32_t freq) {
    const uint32_t capacity_target = I2S_DMA_BUFFER_SIZE / 2;
    uint32_t size = GetDmaSamplesHaveInternal();
    int32_t diff =  ((int32_t)size - (int32_t)capacity_target) * (int32_t)freq / (int32_t)capacity_target;
    int32_t fb_th = (int32_t)freq >> 7;  // 0.78% of freq
    if (diff < -fb_th) diff = -fb_th;
    else if (diff > fb_th) diff = fb_th;
    // // if (size < capacity_target) return -1;
    // else if (size > capacity_target) return 1;
    // else return 0;
    return diff;
}
