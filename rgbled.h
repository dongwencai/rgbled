#ifndef __RGBLED_H__
#define __RGBLED_H__

#define     RSTR    0x00
#define     GCR     0x01
#define     ISR     0x02
#define     CTR    0x30
#define     LCFG0   0x31
#define     LCFG1   0x32
#define     LCFG2   0x33
#define     PWM0    0x34
#define     PWM1    0x35
#define     PWM2    0x36
#define     LED0T0  0x37
#define     LED0T1  0x38
#define     LED0T2  0x39
#define     LED1T0  0x3A
#define     LED1T1  0x3B
#define     LED1T2  0x3C
#define     LED2T0  0x3D
#define     LED2T1  0x3E
#define     LED2T2  0x3F
#define     IADR    0x77

#ifndef uint8
#define uint8   unsigned char
#endif
union lcfg{
    uint8 value;
    struct{
        uint8    IMAX:2;
        uint8    BIT2_3:2;
        uint8    MD:1;
        uint8    FI:1;
        uint8    FO:1;
    }lcfg;
}__attribute__((packed));

union lctr{
    uint8 value;
    struct{
        uint8    LED0:1;
        uint8    LED1:1;
        uint8    LED2:1;
    }lctr;
}__attribute__((packed));

union t1_t2{
    uint8   value;
    struct{
        uint8    T2:3;
        uint8    BIT3:1;
        uint8    T1:3;
    }t1_t2;
}__attribute__((packed));

union t3_t4{
    uint8 value;
    struct{
        uint8    T4:3;
        uint8    BIT3:1;
        uint8    T3:3;
    }t3_t4;
}__attribute__((packed));

union t0{
    uint8   value;
    struct{
        uint8    REPEAT:4;
        uint8    T0:4;
    }t0;
}__attribute__((packed));


struct  ledcfg{
    union lcfg LCFG;
    union lctr LCTR;
    union t1_t2 T1_T2;
    union t3_t4 T3_T4;
    union t0   T0;
    uint8   PWM;
};

#endif
