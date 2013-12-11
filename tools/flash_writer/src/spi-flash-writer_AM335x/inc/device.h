
#define PRCM_BASE            (0x44e00000)
#define CM_PER_SPI0_CLKCTRL  (PRCM_BASE + 0x4c)
#define CM_PER_SPI1_CLKCTRL  (PRCM_BASE + 0x50)
#define CM_PER_I2C0_CLKCTRL (PRCM_BASE + 0x4B8)
#define PRCM_MOD_EN          (0x2)

#define CFG_MOD_BASE       (0x44E10000)      //REVISIT
#define SPI0_SCLK_OFF      (0x950)
#define SPI0_D0_OFF        (0x954)
#define SPI0_D1_OFF        (0x958)
#define SPI0_CS0_OFF       (0x95c)
#define SPI1_SCLK_OFF      (0x990)
#define SPI1_D0_OFF        (0x994)
#define SPI1_D1_OFF        (0x998)
#define SPI1_CS0_OFF       (0x99c)
#define I2C0_SD0_OFF       (0x988)
#define I2C0_SCL_OFF       (0x98c)

#define MODE(val)        (val << 0)       

#define PULL_UD_DISABLE  (1 << 3) /* PULL UP/DOWN disabled */ 	
#define PULL_TYPESEL     (1 << 4) /* PULL UP Select */
#define RXACTIVE         (1 << 5)
#define SLEW_CTRL        (1 << 7)
#define PULL_UP_EN       (PULL_TYPESEL) /* PUL UP Select, PULL_UD_EN = 0 */

typedef struct  pin_muxing {
	Uint32 offset;
	Uint32 val;
}pin_muxing_t, *pin_muxing_handle;

