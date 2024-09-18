#ifndef CAF1229D_CEFE_4E56_8A35_34A403244EAF
#define CAF1229D_CEFE_4E56_8A35_34A403244EAF

#define SSD1680_WAVESHARE_2IN13_V2_LUT_FULL_REFRESH_2 ((uint8_t[]) { \
        0x80,0x60,0x40,0x00,0x00,0x00,0x00,          /* //LUT0: BB:     VS 0 ~7 */ \
    0x10,0x60,0x20,0x00,0x00,0x00,0x00,             /* //LUT1: BW:     VS 0 ~7 */ \
    0x80,0x60,0x40,0x00,0x00,0x00,0x00,             /* //LUT2: WB:     VS 0 ~7 */ \
    0x10,0x60,0x20,0x00,0x00,0x00,0x00,             /* //LUT3: WW:     VS 0 ~7 */ \
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,             /* //LUT4: VCOM:   VS 0 ~7 */ \
 /* -- */ \
    0x03,0x03,0x00,0x00,0x02,                       /* // TP0 A~D RP0 */ \
    0x09,0x09,0x00,0x00,0x02,                       /* // TP1 A~D RP1 */ \
    0x03,0x03,0x00,0x00,0x02,                       /* // TP2 A~D RP2 */ \
    0x00,0x00,0x00,0x00,0x00,                       /* // TP3 A~D RP3 */ \
    0x00,0x00,0x00,0x00,0x00,                       /* // TP4 A~D RP4 */ \
    0x00,0x00,0x00,0x00,0x00,                       /* // TP5 A~D RP5 */ \
    0x00,0x00,0x00,0x00,0x00,                       /* // TP6 A~D RP6 */ \
  /* --- */ \
    0x15,0x41,0xA8,0x32,0x30,0x0A, \
})

// 0x00 ~ 0xff   blink 1 time ~ 256 times to refresh
#define SSD1680_WAVESHARE_2IN13_REFRESH_TIME_O           0x01
#define SSD1680_WAVESHARE_2IN13_V2_LUT_FULL_REFRESH_O ((const uint8_t[]) { \
        /* LUT 0 VS Group 0~11 */ \
        0x80, 0x48, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 1 VS Group 0~11 */ \
        0x40, 0x48, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 2 VS Group 0~11, keep the same as LUT0 for Black-White e-Paper */ \
        0x80, 0x48, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 3 VS Group 0~11, keep the same as LUT1 for Black-White e-Paper */ \
        0x40, 0x48, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 4 VS Group 0~11, seems useless, just keep all zero */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* --- */ \
        /* Only Group0~2 are used */ \
        /* Group 0  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0xA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 1  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0xA, 0x2, 0x00, 0xA, 0x2, 0x00, SSD1680_WAVESHARE_2IN13_REFRESH_TIME_O, \
        /* Group 2  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 3  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 4  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 5  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 6  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 7  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 8  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 9  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 10 TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 11 TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* --- */ \
        0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, \
        /* --- Other register params, do not transfer together with data above */ \
        0x22, 0x17, 0x41, 0x00, 0x32, 0x20 \
})

// 0x00 ~ 0xff   blink 1 time ~ 256 times to refresh
#define SSD1680_WAVESHARE_2IN13_REFRESH_TIME          0x02
#define SSD1680_WAVESHARE_2IN13_V2_LUT_FULL_REFRESH ((const uint8_t[]) { \
        /* LUT 0 VS Group 0~11 */ \
        0x80, 0x4A, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 1 VS Group 0~11 */ \
        0x40, 0x4A, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 2 VS Group 0~11, keep the same as LUT0 for Black-White e-Paper */ \
        0x80, 0x4A, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 3 VS Group 0~11, keep the same as LUT1 for Black-White e-Paper */ \
        0x40, 0x4A, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 4 VS Group 0~11, seems useless, just keep all zero */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* --- */ \
        /* Only Group0~2 are used */ \
        /* Group 0  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0xA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 1  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0xA, 0x2, 0x00, 0xA, 0x2, 0x00, SSD1680_WAVESHARE_2IN13_REFRESH_TIME, \
        /* Group 2  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 3  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 4  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 5  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 6  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 7  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 8  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 9  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 10 TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 11 TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* --- */ \
        0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, \
        /* --- Other register params, do not transfer together with data above */ \
        0x22, 0x17, 0x41, 0x00, 0x32, 0x36 \
})

#define SSD1680_WAVESHARE_2IN13_V2_LUT_FAST_REFRESH_O ((const uint8_t[]) { \
        /* LUT 0 VS Group 0~11 */ \
        0x80, 0x48, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 1 VS Group 0~11 */ \
        0x40, 0x48, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 2 VS Group 0~11, keep the same as LUT0 for Black-White e-Paper */ \
        0x80, 0x48, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 3 VS Group 0~11, keep the same as LUT1 for Black-White e-Paper */ \
        0x40, 0x48, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 4 VS Group 0~11, seems useless, just keep all zero */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* --- */ \
        /* Only Group0~2 are used */ \
        /* Group 0  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 1  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 2  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 3  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 4  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 5  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 6  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 7  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 8  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 9  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 10 TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 11 TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* --- */ \
        0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, \
        /* --- Other register params, do not transfer together with data above */ \
        0x22, 0x17, 0x41, 0x00, 0x32, 0x20 \
})
#define SSD1680_WAVESHARE_2IN13_V2_LUT_FAST_REFRESH_1 ((const uint8_t[]) { \
        /* LUT 0 VS Group 0~11 */ \
        0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 1 VS Group 0~11 */ \
        0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 2 VS Group 0~11, keep the same as LUT0 for Black-White e-Paper */ \
        0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 3 VS Group 0~11, keep the same as LUT1 for Black-White e-Paper */ \
        0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 4 VS Group 0~11, seems useless, just keep all zero */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* --- */ \
        /* Only Group0~2 are used */ \
        /* Group 0  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 1  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 2  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 3  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 4  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 5  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 6  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 7  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 8  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 9  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 10 TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 11 TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* --- */ \
        0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, \
        /* --- Other register params, do not transfer together with data above */ \
        0x22, 0x17, 0x41, 0x00, 0x32, 0x36 \
})

#define SSD1680_WAVESHARE_2IN13_V2_LUT_FAST_REFRESH_2 ((const uint8_t[]) { \
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,             /* //LUT0: BB:     VS 0 ~7 */ \
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,             /* //LUT1: BW:     VS 0 ~7 */ \
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,             /* //LUT2: WB:     VS 0 ~7 */ \
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,             /* //LUT3: WW:     VS 0 ~7 */ \
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,             /* //LUT4: VCOM:   VS 0 ~7 */ \
/*  */ \
    0x0A, 0x00, 0x00, 0x00, 0x00,                       /* // TP0 A~D RP0 */ \
    0x00, 0x00, 0x00, 0x00, 0x00,                       /* // TP1 A~D RP1 */ \
    0x00, 0x00, 0x00, 0x00, 0x00,                       /* // TP2 A~D RP2 */ \
    0x00, 0x00, 0x00, 0x00, 0x00,                       /* // TP4 A~D RP4 */ \
    0x00, 0x00, 0x00, 0x00, 0x00,                       /* // TP3 A~D RP3 */ \
    0x00, 0x00, 0x00, 0x00, 0x00,                       /* // TP5 A~D RP5 */ \
    0x00, 0x00, 0x00, 0x00, 0x00,                       /* // TP6 A~D RP6 */ \
/*  */ \
    0x15, 0x41, 0xA8, 0x32, 0x30, 0x0A, \
})

// NOTE: After several refreshes using SSD1680_WAVESHARE_2IN13_V2_LUT_FAST_REFRESH, you may notice the WHITE color
// goes GRAY and contrast decrease a lot. Use the LUT below to avoid that issue.
// NOTE: The LUT below will have the source output "keep previous output before power off", so the service life may be affected.
#define SSD1680_WAVESHARE_2IN13_V2_LUT_FAST_REFRESH_KEEP_O ((const uint8_t[]) { \
        /* LUT 0 VS Group 0~11 */ \
        0x80, 0x48, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 1 VS Group 0~11 */ \
        0x40, 0x48, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 2 VS Group 0~11, keep the same as LUT0 for Black-White e-Paper */ \
        0x80, 0x48, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 3 VS Group 0~11, keep the same as LUT1 for Black-White e-Paper */ \
        0x40, 0x48, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 4 VS Group 0~11, seems useless, just keep all zero */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* --- */ \
        /* Only Group0~2 are used */ \
        /* Group 0  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 1  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 2  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 3  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 4  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 5  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 6  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 7  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 8  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 9  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 10 TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 11 TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* --- */ \
        0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, \
        /* --- Other register params, do not transfer together with data above */ \
        0x07, 0x17, 0x41, 0x00, 0x32, 0x20 \
})

// NOTE: After several refreshes using SSD1680_WAVESHARE_2IN13_V2_LUT_FAST_REFRESH, you may notice the WHITE color
// goes GRAY and contrast decrease a lot. Use the LUT below to avoid that issue.
// NOTE: The LUT below will have the source output "keep previous output before power off", so the service life may be affected.
#define SSD1680_WAVESHARE_2IN13_V2_LUT_FAST_REFRESH_KEEP_1 ((const uint8_t[]) { \
        /* LUT 0 VS Group 0~11 */ \
        0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 1 VS Group 0~11 */ \
        0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 2 VS Group 0~11, keep the same as LUT0 for Black-White e-Paper */ \
        0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 3 VS Group 0~11, keep the same as LUT1 for Black-White e-Paper */ \
        0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* LUT 4 VS Group 0~11, seems useless, just keep all zero */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* --- */ \
        /* Only Group0~2 are used */ \
        /* Group 0  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 1  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, \
        /* Group 2  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 3  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 4  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 5  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 6  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 7  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 8  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 9  TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 10 TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* Group 11 TP[*A] TP[*B] SR[*AB] TP[*C] TP[*D] SR[*CD] RP[*] */ \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        /* --- */ \
        0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, \
        /* --- Other register params, do not transfer together with data above */ \
        0x07, 0x17, 0x41, 0x00, 0x32, 0x36 \
})

// this is lut from GxEPD2 ESP GPS Logger Library
#define PARTIAL_UPDATE_LUT_SIZE 159
#define PARTIAL_UPDATE_LUT_23 (const uint8_t[]) { \
        0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
      0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  \
      0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  \
      0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  \
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  \
      0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
      0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, \
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, \
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
      0x22, 0x22, 0x22, 0x22, 0x22, 0x22,     \
      0x0, 0x0, 0x0,                              \
      /* --- Other register params, do not transfer together with data above */ \
        0x07, 0x17, 0x41, 0x00, 0x32, 0x20 \
};


#endif /* CAF1229D_CEFE_4E56_8A35_34A403244EAF */
