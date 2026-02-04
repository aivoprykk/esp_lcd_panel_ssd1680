#ifndef SSD168X_WAVESHARE_LUT_H
#define SSD168X_WAVESHARE_LUT_H

// --- Common Macros ---
#define EMPTY_VS 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define EMPTY_GROUP 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define GRP_C(mode) mode, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define GRP_3_11 EMPTY_GROUP, EMPTY_GROUP, EMPTY_GROUP, EMPTY_GROUP, EMPTY_GROUP, EMPTY_GROUP, EMPTY_GROUP, EMPTY_GROUP, EMPTY_GROUP
#define GRP_2_11(mode) GRP_C(mode), GRP_3_11

// --- SSD1680 Specific Macros ---
#define VS1 0x80, 0x4A, 0x40, EMPTY_VS, 0x40, 0x4A, 0x80, EMPTY_VS, 0x80, 0x4A, 0x40, EMPTY_VS, 0x40, 0x4A, 0x80, EMPTY_VS, LUTE, EMPTY_VS
#define VS2 0x00, 0x40, 0x00, EMPTY_VS, 0x80, 0x80, 0x00, EMPTY_VS, 0x40, 0x40, 0x00, EMPTY_VS, 0x00, 0x80, 0x00, EMPTY_VS, LUTE, EMPTY_VS
#define GRP_FULL_0_2(time, mode) 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x02, 0x00, 0x0a, 0x02, 0x00, time, GRP_C(mode)
#define GRP_FULL_0(time, mode) GRP_FULL_0_2(time, mode), GRP_3_11

// --- Common Macros ---
#define LUT0 0x80, 0x48, 0x40
#define LUT1 0x40, 0x48, 0x80
#define LUT2 0x80, 0x48, 0x40
#define LUT3 0x40, 0x48, 0x80
#define LUTE 0x00, 0x00, 0x00
#define GRP_F0 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define GRP_F1 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define GRP_F2 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define VSC \
        LUT0, EMPTY_VS, \
        LUT1, EMPTY_VS, \
        LUT2, EMPTY_VS, \
        LUT2, EMPTY_VS, \
        LUTE, EMPTY_VS
#define PHASE_CTRLC 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00
#define PHASE_CTRL1 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00
#define KEEP_BIT 0x22
#define OTHER_0(mode) mode, 0x17, 0x41, 0x00, 0x32, 0x20
#define OTHER_1(mode) mode, 0x17, 0x41, 0x00, 0x32, 0x36

// --- LUTs for SSD1680 2.13" ---
#define _1680_LUT_FULL_REFRESH_0 ((uint8_t[]) { \
        VSC, \
        GRP_FULL_0(0x01, 0x0a), \
        PHASE_CTRLC, \
        OTHER_0(KEEP_BIT) \
})
#define _1680_LUT_FULL_REFRESH_1 ((uint8_t[]) { \
        VS1, \
        GRP_FULL_0(0x01, 0x0a), \
        PHASE_CTRLC, \
        OTHER_1(KEEP_BIT) \
})
#define _1680_LUT_FAST_REFRESH_0 ((uint8_t[]) { \
        VSC, \
        GRP_F0, \
        GRP_F1, \
        GRP_F2, \
        GRP_3_11, \
        PHASE_CTRLC, \
        OTHER_0(KEEP_BIT) \
})
#define _1680_LUT_FAST_REFRESH_1 ((uint8_t[]) { \
        VS2, \
        GRP_C(0x08), \
        GRP_C(0x1), \
        GRP_2_11(0x1), \
        PHASE_CTRLC, \
        OTHER_1(KEEP_BIT) \
})

// --- LUTs for SSD1681 1.54" ---
#define _1681_REFRESH_TIME           0x01
#define _1681_LUT_FULL_REFRESH ((uint8_t[]) { \
        VSC, \
        GRP_C(0xa), \
        0xA, 0x2, 0x00, 0xA, 0x2, 0x00, _1681_REFRESH_TIME, \
        GRP_2_11(0x0a), \
        PHASE_CTRLC, \
        OTHER_0(0x22) \
})
#define _1681_LUT_FAST_REFRESH ((uint8_t[]) { \
        VSC, \
        GRP_F0, \
        GRP_F1, \
        GRP_F2, \
        GRP_3_11, \
        PHASE_CTRLC, \
        OTHER_0(0x22) \
})
#define _1681_LUT_FAST_REFRESH_KEEP ((uint8_t[]) { \
        VSC, \
        GRP_F0, \
        GRP_F1, \
        GRP_F2, \
        GRP_3_11, \
        PHASE_CTRLC, \
        OTHER_0(0x07) \
})

#define PARTIAL_UPDATE_LUT_SIZE 159

#endif // SSD168X_WAVESHARE_LUT_H
