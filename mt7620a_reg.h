#ifndef  __MT7620A_REG_H__
#define  __MT7620A_REG_H__

#define CONFIG_RALINK_MT7620
#include <asm/rt2880/rt_mmap.h>
#include <asm/rt2880/surfboardint.h>

#define GPIO_MODE_OFFSET    0x0060
#define GPIO_39_24_DATA_OFFSET 0x0048
#define GPIO_39_24_DIR_OFFSET   0x004C
#define GPIO_39_24_INT_OFFSET 0x0038
#define GPIO_39_24_EDGE_OFFSET  0x003C
#define GPIO_39_24_RMASK_OFFSET 0x0040
#define GPIO_39_24_FMASK_OFFSET 0x0044

#define GPIO_71_40_DATA_OFFSET  0x0070
#define GPIO_71_40_DIR_OFFSET   0x0074
#define GPIO_71_40_INT_OFFSET   0x0060
#define GPIO_71_40_EDGE_OFFSET  0x0064
#define GPIO_71_40_RMASK_OFFSET 0x0068
#define GPIO_71_40_FMASK_OFFSET 0x006C

#define INTENA_OFFSET   0x0034

#define GPIO_MODE  (RALINK_SYSCTL_BASE + GPIO_MODE_OFFSET)

#define GPIO_39_24_DATA     (RALINK_PIO_BASE + GPIO_39_24_DATA_OFFSET)
#define GPIO_39_24_DIR      (RALINK_PIO_BASE + GPIO_39_24_DIR_OFFSET)
#define GPIO_39_24_INT      (RALINK_PIO_BASE + GPIO_39_24_INT_OFFSET)
#define GPIO_39_24_EDGE     (RALINK_PIO_BASE + GPIO_39_24_EDGE_OFFSET)
#define GPIO_39_24_RMASK    (RALINK_PIO_BASE + GPIO_39_24_RMASK_OFFSET)
#define GPIO_39_24_FMASK    (RALINK_PIO_BASE + GPIO_39_24_FMASK_OFFSET)
#define INTENA              (RALINK_INTCL_BASE + INTENA_OFFSET)

#define GPIO_71_40_DATA     (RALINK_PIO_BASE + GPIO_71_40_DATA_OFFSET)
#define GPIO_71_40_DIR      (RALINK_PIO_BASE + GPIO_71_40_DIR_OFFSET)
#define GPIO_71_40_INT      (RALINK_PIO_BASE + GPIO_71_40_INT_OFFSET)
#define GPIO_71_40_EDGE     (RALINK_PIO_BASE + GPIO_71_40_EDGE_OFFSET)
#define GPIO_71_40_RMASK    (RALINK_PIO_BASE + GPIO_71_40_RMASK_OFFSET)
#define GPIO_71_40_FMASK    (RALINK_PIO_BASE + GPIO_71_40_FMASK_OFFSET)

#endif