#include "ch32v30x_misc.h"
#include "debug.h"
#include "tick.h"
#include "usb/usb_impl.h"
#include "usbd.h"
#include "codec.h"
#include "audio_dsp.h"
#include "codec_i2s.h"
#include "uac_param.h"

static uint32_t feedback_fs_;

// static void updateFeedback(void) {
//     // 1ms throttle: use the I2S TX buffer water level to tune the USB feedback,
//     // but do not generate or change the actual slave I2S clock. The external
//     // audio master decides the real BCLK/LRCLK timing.
//     static uint32_t last_fb_tick;
//     uint32_t now = Tick_GetTick();
//     if (now - last_fb_tick >= 1) {
//         last_fb_tick = now;
//         const struct UacFeedbackRate* rate = Codec_GetFeedbackRate();
//         int32_t diff = CodecI2s_GetCurrentSizeDiffFromCenter();
//         uint32_t sr = (diff < 0) ? rate->ceil : (diff > 0) ? rate->floor : rate->normal;
//         feedback_fs_ = sr;
//         UsbImpl_SetFeedbackSr(sr);
//     }
// }

enum HidCommand {
    kHidCommand_Update = 0,
    kHidCommand_Save = 1
};

static void handleHid(void) {
    if (!UsbHid_HasData()) {
        return;
    }
    
    const uint32_t* hid_data = UsbHid_GetData();
    switch (hid_data[0]) {
        case kHidCommand_Update: {
            struct UacParam* param = Codec_GetUacParam();
            memcpy(&param->sr48k, hid_data + 1, sizeof(struct UacParam) - sizeof(uint32_t));
            UacParam_Clamp(param);   // 限制到合法范围
        }
            break;

        case kHidCommand_Save: {
            if (UacParam_Write(Codec_GetUacParam())) {
                printf("uac param saved\n\r");
            }
            else {
                printf("uac param save failed\n\r");
            }
        }
            break;
    }

    UsbHid_BeginRecv();
}

int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    Tick_Init();
    Usbd_Init();
    bool inited = Codec_Init();

    Usbd_Connect();
    uint32_t tick = Tick_GetTick();
    for (;;) {
        Codec_Handler();
        AudioDsp_Handler();
        // updateFeedback();
        handleHid();

        uint32_t now = Tick_GetTick();
        if (now - tick > 100) {
            // if (!inited) {
            //     printf("codec inited failed\n\r");
            // }
            tick = now;

            /*
             * 16-column diagnostic log matching whatido.md §5.1:
             *  1  DMA read position
             *  2  Software ring write position
             *  3  Estimated ring samples present
             *  4  Estimated ring free samples
             *  5  Min ring samples present in this interval (reset each period)
             *  6  Max ring samples present in this interval (reset each period)
             *  7  OUT cumulative byte count
             *  8  Zero-length OUT cumulative count
             *  9  SET_CUR count
             * 10  Same-rate SET_CUR ignored count
             * 11  DMA underflow sample estimate (cumulative)
             * 12  OUT packets received since previous SOF snapshot
             * 13  OUT bytes received since previous SOF snapshot
             * 14  OUT-missing-SOF cumulative count
             * 15  Short OUT packet cumulative count (below microframe minimum)
             * 16  Maximum short/zero-packet run length
             */
            printf("DBG,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
                   CodecI2s_GetDmaReadPos(),
                   CodecI2s_GetDmaWritePos(),
                   CodecI2s_GetDmaSamplesHave(),
                   CodecI2s_GetFreeSpace(),
                   CodecI2s_GetDmaMinSamplesHave(),
                   CodecI2s_GetDmaMaxSamplesHave(),
                   UsbUac2_GetOutByteCount(),
                   UsbUac2_GetOutZeroCount(),
                   UsbUac2_GetSetCurCount(),
                   UsbUac2_GetSetCurIgnoredCount(),
                   CodecI2s_GetDmaUnderflowSamples(),
                   UsbUac2_GetSofOutPackets(),
                   UsbUac2_GetSofOutBytes(),
                   UsbUac2_GetMissingSofCount(),
                   UsbUac2_GetShortOutCount(),
                   UsbUac2_GetShortOutMaxRun());

            /* Reset per-interval min/max for the next 100 ms window */
            CodecI2s_ResetIntervalStats();
        //     printf("i2s dma free space[%ld/%d]    sr[%ld/%ld]\n\r",
        //         CodecI2s_GetFreeSpace(), I2S_DMA_BUFFER_SIZE,
        //         feedback_fs_, Codec_GetSampleRate()
        //     );
        }
    }
}
