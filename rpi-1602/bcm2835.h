// bcm2835.h
// Defines for BCM2835
#ifndef BCM2835_H
#define BCM2835_H

/// This means pin HIGH, true, 3.3volts on a pin.
#define HIGH 0x1
/// This means pin LOW, false, 0volts on a pin.
#define LOW  0x0

/// Speed of the core clock core_clk
#define BCM2835_CORE_CLK_HZ				250000000	///< 250 MHz

// Physical addresses for various peripheral register sets
/// Base Physical Address of the BCM 2835 peripheral registers
#define BCM2835_PERI_BASE               0x20000000
/// Base Physical Address of the System Timer registers
#define BCM2835_ST_BASE			(BCM2835_PERI_BASE + 0x3000)
/// Base Physical Address of the Pads registers
#define BCM2835_GPIO_PADS               (BCM2835_PERI_BASE + 0x100000)
/// Base Physical Address of the Clock/timer registers
#define BCM2835_CLOCK_BASE              (BCM2835_PERI_BASE + 0x101000)
/// Base Physical Address of the GPIO registers
#define BCM2835_GPIO_BASE               (BCM2835_PERI_BASE + 0x200000)
/// Base Physical Address of the SPI0 registers
#define BCM2835_SPI0_BASE               (BCM2835_PERI_BASE + 0x204000)
/// Base Physical Address of the BSC0 registers
#define BCM2835_BSC0_BASE 		(BCM2835_PERI_BASE + 0x205000)
/// Base Physical Address of the PWM registers
#define BCM2835_GPIO_PWM                (BCM2835_PERI_BASE + 0x20C000)
 /// Base Physical Address of the BSC1 registers
#define BCM2835_BSC1_BASE		(BCM2835_PERI_BASE + 0x804000)


/// Size of memory page on RPi
#define BCM2835_PAGE_SIZE               (4*1024)
/// Size of memory block on RPi
#define BCM2835_BLOCK_SIZE              (4*1024)

// Defines for GPIO
// The BCM2835 has 54 GPIO pins.
//      BCM2835 data sheet, Page 90 onwards.
/// GPIO register offsets from BCM2835_GPIO_BASE. Offsets into the GPIO Peripheral block in bytes per 6.1 Register View
#define BCM2835_GPFSEL0                      0x0000 ///< GPIO Function Select 0
#define BCM2835_GPFSEL1                      0x0004 ///< GPIO Function Select 1
#define BCM2835_GPFSEL2                      0x0008 ///< GPIO Function Select 2
#define BCM2835_GPFSEL3                      0x000c ///< GPIO Function Select 3
#define BCM2835_GPFSEL4                      0x0010 ///< GPIO Function Select 4
#define BCM2835_GPFSEL5                      0x0014 ///< GPIO Function Select 5
#define BCM2835_GPSET0                       0x001c ///< GPIO Pin Output Set 0
#define BCM2835_GPSET1                       0x0020 ///< GPIO Pin Output Set 1
#define BCM2835_GPCLR0                       0x0028 ///< GPIO Pin Output Clear 0
#define BCM2835_GPCLR1                       0x002c ///< GPIO Pin Output Clear 1
#define BCM2835_GPLEV0                       0x0034 ///< GPIO Pin Level 0
#define BCM2835_GPLEV1                       0x0038 ///< GPIO Pin Level 1
#define BCM2835_GPEDS0                       0x0040 ///< GPIO Pin Event Detect Status 0
#define BCM2835_GPEDS1                       0x0044 ///< GPIO Pin Event Detect Status 1
#define BCM2835_GPREN0                       0x004c ///< GPIO Pin Rising Edge Detect Enable 0
#define BCM2835_GPREN1                       0x0050 ///< GPIO Pin Rising Edge Detect Enable 1
#define BCM2835_GPFEN0                       0x0058 ///< GPIO Pin Falling Edge Detect Enable 0
#define BCM2835_GPFEN1                       0x005c ///< GPIO Pin Falling Edge Detect Enable 1
#define BCM2835_GPHEN0                       0x0064 ///< GPIO Pin High Detect Enable 0
#define BCM2835_GPHEN1                       0x0068 ///< GPIO Pin High Detect Enable 1
#define BCM2835_GPLEN0                       0x0070 ///< GPIO Pin Low Detect Enable 0
#define BCM2835_GPLEN1                       0x0074 ///< GPIO Pin Low Detect Enable 1
#define BCM2835_GPAREN0                      0x007c ///< GPIO Pin Async. Rising Edge Detect 0
#define BCM2835_GPAREN1                      0x0080 ///< GPIO Pin Async. Rising Edge Detect 1
#define BCM2835_GPAFEN0                      0x0088 ///< GPIO Pin Async. Falling Edge Detect 0
#define BCM2835_GPAFEN1                      0x008c ///< GPIO Pin Async. Falling Edge Detect 1
#define BCM2835_GPPUD                        0x0094 ///< GPIO Pin Pull-up/down Enable
#define BCM2835_GPPUDCLK0                    0x0098 ///< GPIO Pin Pull-up/down Enable Clock 0
#define BCM2835_GPPUDCLK1                    0x009c ///< GPIO Pin Pull-up/down Enable Clock 1

/// \brief bcm2835PortFunction
/// Port function select modes for bcm2835_gpio_fsel()
typedef enum
{
    BCM2835_GPIO_FSEL_INPT  = 0b000,   ///< Input
    BCM2835_GPIO_FSEL_OUTP  = 0b001,   ///< Output
    BCM2835_GPIO_FSEL_ALT0  = 0b100,   ///< Alternate function 0
    BCM2835_GPIO_FSEL_ALT1  = 0b101,   ///< Alternate function 1
    BCM2835_GPIO_FSEL_ALT2  = 0b110,   ///< Alternate function 2
    BCM2835_GPIO_FSEL_ALT3  = 0b111,   ///< Alternate function 3
    BCM2835_GPIO_FSEL_ALT4  = 0b011,   ///< Alternate function 4
    BCM2835_GPIO_FSEL_ALT5  = 0b010,   ///< Alternate function 5
    BCM2835_GPIO_FSEL_MASK  = 0b111    ///< Function select bits mask
} bcm2835FunctionSelect;

/// \brief bcm2835PUDControl
/// Pullup/Pulldown defines for bcm2835_gpio_pud()
typedef enum
{
    BCM2835_GPIO_PUD_OFF     = 0b00,   ///< Off ? disable pull-up/down
    BCM2835_GPIO_PUD_DOWN    = 0b01,   ///< Enable Pull Down control
    BCM2835_GPIO_PUD_UP      = 0b10    ///< Enable Pull Up control
} bcm2835PUDControl;

/// Pad control register offsets from BCM2835_GPIO_PADS
#define BCM2835_PADS_GPIO_0_27               0x002c ///< Pad control register for pads 0 to 27
#define BCM2835_PADS_GPIO_28_45              0x0030 ///< Pad control register for pads 28 to 45
#define BCM2835_PADS_GPIO_46_53              0x0034 ///< Pad control register for pads 46 to 53

/// Pad Control masks
#define BCM2835_PAD_PASSWRD                  (0x5A << 24)  ///< Password to enable setting pad mask
#define BCM2835_PAD_SLEW_RATE_UNLIMITED      0x10 ///< Slew rate unlimited
#define BCM2835_PAD_HYSTERESIS_ENABLED       0x08 ///< Hysteresis enabled
#define BCM2835_PAD_DRIVE_2mA                0x00 ///< 2mA drive current
#define BCM2835_PAD_DRIVE_4mA                0x01 ///< 4mA drive current
#define BCM2835_PAD_DRIVE_6mA                0x02 ///< 6mA drive current
#define BCM2835_PAD_DRIVE_8mA                0x03 ///< 8mA drive current
#define BCM2835_PAD_DRIVE_10mA               0x04 ///< 10mA drive current
#define BCM2835_PAD_DRIVE_12mA               0x05 ///< 12mA drive current
#define BCM2835_PAD_DRIVE_14mA               0x06 ///< 14mA drive current
#define BCM2835_PAD_DRIVE_16mA               0x07 ///< 16mA drive current

/// \brief bcm2835PadGroup
/// Pad group specification for bcm2835_gpio_pad()
typedef enum
{
    BCM2835_PAD_GROUP_GPIO_0_27         = 0, ///< Pad group for GPIO pads 0 to 27
    BCM2835_PAD_GROUP_GPIO_28_45        = 1, ///< Pad group for GPIO pads 28 to 45
    BCM2835_PAD_GROUP_GPIO_46_53        = 2  ///< Pad group for GPIO pads 46 to 53
} bcm2835PadGroup;

/// \brief GPIO Pin Numbers
///
/// Here we define Raspberry Pin GPIO pins on P1 in terms of the underlying BCM GPIO pin numbers.
/// These can be passed as a pin number to any function requiring a pin.
/// Not all pins on the RPi 26 bin IDE plug are connected to GPIO pins
/// and some can adopt an alternate function.
/// RPi version 2 has some slightly different pinouts, and these are values RPI_V2_*.
/// RPi B+ has yet differnet pinouts and these are defined in RPI_BPLUS_*.
/// At bootup, pins 8 and 10 are set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
/// When SPI0 is in use (ie after bcm2835_spi_begin()), SPI0 pins are dedicated to SPI
/// and cant be controlled independently
typedef enum
{
    RPI_GPIO_P1_03        =  0,  ///< Version 1, Pin P1-03
    RPI_GPIO_P1_05        =  1,  ///< Version 1, Pin P1-05
    RPI_GPIO_P1_07        =  4,  ///< Version 1, Pin P1-07
    RPI_GPIO_P1_08        = 14,  ///< Version 1, Pin P1-08, defaults to alt function 0 UART0_TXD
    RPI_GPIO_P1_10        = 15,  ///< Version 1, Pin P1-10, defaults to alt function 0 UART0_RXD
    RPI_GPIO_P1_11        = 17,  ///< Version 1, Pin P1-11
    RPI_GPIO_P1_12        = 18,  ///< Version 1, Pin P1-12, can be PWM channel 0 in ALT FUN 5
    RPI_GPIO_P1_13        = 21,  ///< Version 1, Pin P1-13
    RPI_GPIO_P1_15        = 22,  ///< Version 1, Pin P1-15
    RPI_GPIO_P1_16        = 23,  ///< Version 1, Pin P1-16
    RPI_GPIO_P1_18        = 24,  ///< Version 1, Pin P1-18
    RPI_GPIO_P1_19        = 10,  ///< Version 1, Pin P1-19, MOSI when SPI0 in use
    RPI_GPIO_P1_21        =  9,  ///< Version 1, Pin P1-21, MISO when SPI0 in use
    RPI_GPIO_P1_22        = 25,  ///< Version 1, Pin P1-22
    RPI_GPIO_P1_23        = 11,  ///< Version 1, Pin P1-23, CLK when SPI0 in use
    RPI_GPIO_P1_24        =  8,  ///< Version 1, Pin P1-24, CE0 when SPI0 in use
    RPI_GPIO_P1_26        =  7,  ///< Version 1, Pin P1-26, CE1 when SPI0 in use

    // RPi Version 2
    RPI_V2_GPIO_P1_03     =  2,  ///< Version 2, Pin P1-03
    RPI_V2_GPIO_P1_05     =  3,  ///< Version 2, Pin P1-05
    RPI_V2_GPIO_P1_07     =  4,  ///< Version 2, Pin P1-07
    RPI_V2_GPIO_P1_08     = 14,  ///< Version 2, Pin P1-08, defaults to alt function 0 UART0_TXD
    RPI_V2_GPIO_P1_10     = 15,  ///< Version 2, Pin P1-10, defaults to alt function 0 UART0_RXD
    RPI_V2_GPIO_P1_11     = 17,  ///< Version 2, Pin P1-11
    RPI_V2_GPIO_P1_12     = 18,  ///< Version 2, Pin P1-12, can be PWM channel 0 in ALT FUN 5
    RPI_V2_GPIO_P1_13     = 27,  ///< Version 2, Pin P1-13
    RPI_V2_GPIO_P1_15     = 22,  ///< Version 2, Pin P1-15
    RPI_V2_GPIO_P1_16     = 23,  ///< Version 2, Pin P1-16
    RPI_V2_GPIO_P1_18     = 24,  ///< Version 2, Pin P1-18
    RPI_V2_GPIO_P1_19     = 10,  ///< Version 2, Pin P1-19, MOSI when SPI0 in use
    RPI_V2_GPIO_P1_21     =  9,  ///< Version 2, Pin P1-21, MISO when SPI0 in use
    RPI_V2_GPIO_P1_22     = 25,  ///< Version 2, Pin P1-22
    RPI_V2_GPIO_P1_23     = 11,  ///< Version 2, Pin P1-23, CLK when SPI0 in use
    RPI_V2_GPIO_P1_24     =  8,  ///< Version 2, Pin P1-24, CE0 when SPI0 in use
    RPI_V2_GPIO_P1_26     =  7,  ///< Version 2, Pin P1-26, CE1 when SPI0 in use

    // RPi Version 2, new plug P5
    RPI_V2_GPIO_P5_03     = 28,  ///< Version 2, Pin P5-03
    RPI_V2_GPIO_P5_04     = 29,  ///< Version 2, Pin P5-04
    RPI_V2_GPIO_P5_05     = 30,  ///< Version 2, Pin P5-05
    RPI_V2_GPIO_P5_06     = 31,  ///< Version 2, Pin P5-06

    // RPi B+ J8 header
    RPI_BPLUS_GPIO_J8_03     =  2,  ///< B+, Pin J8-03
    RPI_BPLUS_GPIO_J8_05     =  3,  ///< B+, Pin J8-05
    RPI_BPLUS_GPIO_J8_07     =  4,  ///< B+, Pin J8-07
    RPI_BPLUS_GPIO_J8_08     = 14,  ///< B+, Pin J8-08, defaults to alt function 0 UART0_TXD
    RPI_BPLUS_GPIO_J8_10     = 15,  ///< B+, Pin J8-10, defaults to alt function 0 UART0_RXD
    RPI_BPLUS_GPIO_J8_11     = 17,  ///< B+, Pin J8-11
    RPI_BPLUS_GPIO_J8_12     = 18,  ///< B+, Pin J8-12, can be PWM channel 0 in ALT FUN 5
    RPI_BPLUS_GPIO_J8_13     = 27,  ///< B+, Pin J8-13
    RPI_BPLUS_GPIO_J8_15     = 22,  ///< B+, Pin J8-15
    RPI_BPLUS_GPIO_J8_16     = 23,  ///< B+, Pin J8-16
    RPI_BPLUS_GPIO_J8_18     = 24,  ///< B+, Pin J8-18
    RPI_BPLUS_GPIO_J8_19     = 10,  ///< B+, Pin J8-19, MOSI when SPI0 in use
    RPI_BPLUS_GPIO_J8_21     =  9,  ///< B+, Pin J8-21, MISO when SPI0 in use
    RPI_BPLUS_GPIO_J8_22     = 25,  ///< B+, Pin J8-22
    RPI_BPLUS_GPIO_J8_23     = 11,  ///< B+, Pin J8-23, CLK when SPI0 in use
    RPI_BPLUS_GPIO_J8_24     =  8,  ///< B+, Pin J8-24, CE0 when SPI0 in use
    RPI_BPLUS_GPIO_J8_26     =  7,  ///< B+, Pin J8-26, CE1 when SPI0 in use
    RPI_BPLUS_GPIO_J8_29     =  5,  ///< B+, Pin J8-29, 
    RPI_BPLUS_GPIO_J8_31     =  6,  ///< B+, Pin J8-31, 
    RPI_BPLUS_GPIO_J8_32     =  12, ///< B+, Pin J8-32, 
    RPI_BPLUS_GPIO_J8_33     =  13, ///< B+, Pin J8-33, 
    RPI_BPLUS_GPIO_J8_35     =  19, ///< B+, Pin J8-35, 
    RPI_BPLUS_GPIO_J8_36     =  16, ///< B+, Pin J8-36, 
    RPI_BPLUS_GPIO_J8_37     =  26, ///< B+, Pin J8-37, 
    RPI_BPLUS_GPIO_J8_38     =  20, ///< B+, Pin J8-38, 
    RPI_BPLUS_GPIO_J8_40     =  21, ///< B+, Pin J8-40, 
} RPiGPIOPin;

// Defines for SPI
// GPIO register offsets from BCM2835_SPI0_BASE. 
// Offsets into the SPI Peripheral block in bytes per 10.5 SPI Register Map
#define BCM2835_SPI0_CS                      0x0000 ///< SPI Master Control and Status
#define BCM2835_SPI0_FIFO                    0x0004 ///< SPI Master TX and RX FIFOs
#define BCM2835_SPI0_CLK                     0x0008 ///< SPI Master Clock Divider
#define BCM2835_SPI0_DLEN                    0x000c ///< SPI Master Data Length
#define BCM2835_SPI0_LTOH                    0x0010 ///< SPI LOSSI mode TOH
#define BCM2835_SPI0_DC                      0x0014 ///< SPI DMA DREQ Controls

// Register masks for SPI0_CS
#define BCM2835_SPI0_CS_LEN_LONG             0x02000000 ///< Enable Long data word in Lossi mode if DMA_LEN is set
#define BCM2835_SPI0_CS_DMA_LEN              0x01000000 ///< Enable DMA mode in Lossi mode
#define BCM2835_SPI0_CS_CSPOL2               0x00800000 ///< Chip Select 2 Polarity
#define BCM2835_SPI0_CS_CSPOL1               0x00400000 ///< Chip Select 1 Polarity
#define BCM2835_SPI0_CS_CSPOL0               0x00200000 ///< Chip Select 0 Polarity
#define BCM2835_SPI0_CS_RXF                  0x00100000 ///< RXF - RX FIFO Full
#define BCM2835_SPI0_CS_RXR                  0x00080000 ///< RXR RX FIFO needs Reading ( full)
#define BCM2835_SPI0_CS_TXD                  0x00040000 ///< TXD TX FIFO can accept Data
#define BCM2835_SPI0_CS_RXD                  0x00020000 ///< RXD RX FIFO contains Data
#define BCM2835_SPI0_CS_DONE                 0x00010000 ///< Done transfer Done
#define BCM2835_SPI0_CS_TE_EN                0x00008000 ///< Unused
#define BCM2835_SPI0_CS_LMONO                0x00004000 ///< Unused
#define BCM2835_SPI0_CS_LEN                  0x00002000 ///< LEN LoSSI enable
#define BCM2835_SPI0_CS_REN                  0x00001000 ///< REN Read Enable
#define BCM2835_SPI0_CS_ADCS                 0x00000800 ///< ADCS Automatically Deassert Chip Select
#define BCM2835_SPI0_CS_INTR                 0x00000400 ///< INTR Interrupt on RXR
#define BCM2835_SPI0_CS_INTD                 0x00000200 ///< INTD Interrupt on Done
#define BCM2835_SPI0_CS_DMAEN                0x00000100 ///< DMAEN DMA Enable
#define BCM2835_SPI0_CS_TA                   0x00000080 ///< Transfer Active
#define BCM2835_SPI0_CS_CSPOL                0x00000040 ///< Chip Select Polarity
#define BCM2835_SPI0_CS_CLEAR                0x00000030 ///< Clear FIFO Clear RX and TX
#define BCM2835_SPI0_CS_CLEAR_RX             0x00000020 ///< Clear FIFO Clear RX 
#define BCM2835_SPI0_CS_CLEAR_TX             0x00000010 ///< Clear FIFO Clear TX 
#define BCM2835_SPI0_CS_CPOL                 0x00000008 ///< Clock Polarity
#define BCM2835_SPI0_CS_CPHA                 0x00000004 ///< Clock Phase
#define BCM2835_SPI0_CS_CS                   0x00000003 ///< Chip Select

/// \brief bcm2835SPIBitOrder SPI Bit order
/// Specifies the SPI data bit ordering for bcm2835_spi_setBitOrder()
typedef enum
{
    BCM2835_SPI_BIT_ORDER_LSBFIRST = 0,  ///< LSB First
    BCM2835_SPI_BIT_ORDER_MSBFIRST = 1   ///< MSB First
}bcm2835SPIBitOrder;

/// \brief SPI Data mode
/// Specify the SPI data mode to be passed to bcm2835_spi_setDataMode()
typedef enum
{
    BCM2835_SPI_MODE0 = 0,  ///< CPOL = 0, CPHA = 0
    BCM2835_SPI_MODE1 = 1,  ///< CPOL = 0, CPHA = 1
    BCM2835_SPI_MODE2 = 2,  ///< CPOL = 1, CPHA = 0
    BCM2835_SPI_MODE3 = 3,  ///< CPOL = 1, CPHA = 1
}bcm2835SPIMode;

/// \brief bcm2835SPIChipSelect
/// Specify the SPI chip select pin(s)
typedef enum
{
    BCM2835_SPI_CS0 = 0,     ///< Chip Select 0
    BCM2835_SPI_CS1 = 1,     ///< Chip Select 1
    BCM2835_SPI_CS2 = 2,     ///< Chip Select 2 (ie pins CS1 and CS2 are asserted)
    BCM2835_SPI_CS_NONE = 3, ///< No CS, control it yourself
} bcm2835SPIChipSelect;

/// \brief bcm2835SPIClockDivider
/// Specifies the divider used to generate the SPI clock from the system clock.
/// Figures below give the divider, clock period and clock frequency.
/// Clock divided is based on nominal base clock rate of 250MHz
/// It is reported that (contrary to the documentation) any even divider may used.
/// The frequencies shown for each divider have been confirmed by measurement
typedef enum
{
    BCM2835_SPI_CLOCK_DIVIDER_65536 = 0,       ///< 65536 = 262.144us = 3.814697260kHz
    BCM2835_SPI_CLOCK_DIVIDER_32768 = 32768,   ///< 32768 = 131.072us = 7.629394531kHz
    BCM2835_SPI_CLOCK_DIVIDER_16384 = 16384,   ///< 16384 = 65.536us = 15.25878906kHz
    BCM2835_SPI_CLOCK_DIVIDER_8192  = 8192,    ///< 8192 = 32.768us = 30/51757813kHz
    BCM2835_SPI_CLOCK_DIVIDER_4096  = 4096,    ///< 4096 = 16.384us = 61.03515625kHz
    BCM2835_SPI_CLOCK_DIVIDER_2048  = 2048,    ///< 2048 = 8.192us = 122.0703125kHz
    BCM2835_SPI_CLOCK_DIVIDER_1024  = 1024,    ///< 1024 = 4.096us = 244.140625kHz
    BCM2835_SPI_CLOCK_DIVIDER_512   = 512,     ///< 512 = 2.048us = 488.28125kHz
    BCM2835_SPI_CLOCK_DIVIDER_256   = 256,     ///< 256 = 1.024us = 976.5625MHz
    BCM2835_SPI_CLOCK_DIVIDER_128   = 128,     ///< 128 = 512ns = = 1.953125MHz
    BCM2835_SPI_CLOCK_DIVIDER_64    = 64,      ///< 64 = 256ns = 3.90625MHz
    BCM2835_SPI_CLOCK_DIVIDER_32    = 32,      ///< 32 = 128ns = 7.8125MHz
    BCM2835_SPI_CLOCK_DIVIDER_16    = 16,      ///< 16 = 64ns = 15.625MHz
    BCM2835_SPI_CLOCK_DIVIDER_8     = 8,       ///< 8 = 32ns = 31.25MHz
    BCM2835_SPI_CLOCK_DIVIDER_4     = 4,       ///< 4 = 16ns = 62.5MHz
    BCM2835_SPI_CLOCK_DIVIDER_2     = 2,       ///< 2 = 8ns = 125MHz, fastest you can get
    BCM2835_SPI_CLOCK_DIVIDER_1     = 1,       ///< 1 = 262.144us = 3.814697260kHz, same as 0/65536
} bcm2835SPIClockDivider;

// Defines for I2C
// GPIO register offsets from BCM2835_BSC*_BASE.
// Offsets into the BSC Peripheral block in bytes per 3.1 BSC Register Map
#define BCM2835_BSC_C 							0x0000 ///< BSC Master Control
#define BCM2835_BSC_S 							0x0004 ///< BSC Master Status
#define BCM2835_BSC_DLEN						0x0008 ///< BSC Master Data Length
#define BCM2835_BSC_A 							0x000c ///< BSC Master Slave Address
#define BCM2835_BSC_FIFO						0x0010 ///< BSC Master Data FIFO
#define BCM2835_BSC_DIV							0x0014 ///< BSC Master Clock Divider
#define BCM2835_BSC_DEL							0x0018 ///< BSC Master Data Delay
#define BCM2835_BSC_CLKT						0x001c ///< BSC Master Clock Stretch Timeout

// Register masks for BSC_C
#define BCM2835_BSC_C_I2CEN 					0x00008000 ///< I2C Enable, 0 = disabled, 1 = enabled
#define BCM2835_BSC_C_INTR 						0x00000400 ///< Interrupt on RX
#define BCM2835_BSC_C_INTT 						0x00000200 ///< Interrupt on TX
#define BCM2835_BSC_C_INTD 						0x00000100 ///< Interrupt on DONE
#define BCM2835_BSC_C_ST 						0x00000080 ///< Start transfer, 1 = Start a new transfer
#define BCM2835_BSC_C_CLEAR_1 					0x00000020 ///< Clear FIFO Clear
#define BCM2835_BSC_C_CLEAR_2 					0x00000010 ///< Clear FIFO Clear
#define BCM2835_BSC_C_READ 						0x00000001 ///<	Read transfer

// Register masks for BSC_S
#define BCM2835_BSC_S_CLKT 						0x00000200 ///< Clock stretch timeout
#define BCM2835_BSC_S_ERR 						0x00000100 ///< ACK error
#define BCM2835_BSC_S_RXF 						0x00000080 ///< RXF FIFO full, 0 = FIFO is not full, 1 = FIFO is full
#define BCM2835_BSC_S_TXE 						0x00000040 ///< TXE FIFO full, 0 = FIFO is not full, 1 = FIFO is full
#define BCM2835_BSC_S_RXD 						0x00000020 ///< RXD FIFO contains data
#define BCM2835_BSC_S_TXD 						0x00000010 ///< TXD FIFO can accept data
#define BCM2835_BSC_S_RXR 						0x00000008 ///< RXR FIFO needs reading (full)
#define BCM2835_BSC_S_TXW 						0x00000004 ///< TXW FIFO needs writing (full)
#define BCM2835_BSC_S_DONE 						0x00000002 ///< Transfer DONE
#define BCM2835_BSC_S_TA 						0x00000001 ///< Transfer Active

#define BCM2835_BSC_FIFO_SIZE   				16 ///< BSC FIFO size

/// \brief bcm2835I2CClockDivider
/// Specifies the divider used to generate the I2C clock from the system clock.
/// Clock divided is based on nominal base clock rate of 250MHz
typedef enum
{
    BCM2835_I2C_CLOCK_DIVIDER_2500   = 2500,      ///< 2500 = 10us = 100 kHz
    BCM2835_I2C_CLOCK_DIVIDER_626    = 626,       ///< 622 = 2.504us = 399.3610 kHz
    BCM2835_I2C_CLOCK_DIVIDER_150    = 150,       ///< 150 = 60ns = 1.666 MHz (default at reset)
    BCM2835_I2C_CLOCK_DIVIDER_148    = 148,       ///< 148 = 59ns = 1.689 MHz
} bcm2835I2CClockDivider;

/// \brief bcm2835I2CReasonCodes
/// Specifies the reason codes for the bcm2835_i2c_write and bcm2835_i2c_read functions.
typedef enum
{
    BCM2835_I2C_REASON_OK   	     = 0x00,      ///< Success
    BCM2835_I2C_REASON_ERROR_NACK    = 0x01,      ///< Received a NACK
    BCM2835_I2C_REASON_ERROR_CLKT    = 0x02,      ///< Received Clock Stretch Timeout
    BCM2835_I2C_REASON_ERROR_DATA    = 0x04,      ///< Not all data is sent / received
} bcm2835I2CReasonCodes;

// Defines for ST
// GPIO register offsets from BCM2835_ST_BASE.
// Offsets into the ST Peripheral block in bytes per 12.1 System Timer Registers
// The System Timer peripheral provides four 32-bit timer channels and a single 64-bit free running counter.
// BCM2835_ST_CLO is the System Timer Counter Lower bits register.
// The system timer free-running counter lower register is a read-only register that returns the current value
// of the lower 32-bits of the free running counter.
// BCM2835_ST_CHI is the System Timer Counter Upper bits register.
// The system timer free-running counter upper register is a read-only register that returns the current value
// of the upper 32-bits of the free running counter.
#define BCM2835_ST_CS 							0x0000 ///< System Timer Control/Status
#define BCM2835_ST_CLO 							0x0004 ///< System Timer Counter Lower 32 bits
#define BCM2835_ST_CHI 							0x0008 ///< System Timer Counter Upper 32 bits

/// @}


// Defines for PWM, word offsets (ie 4 byte multiples)
#define BCM2835_PWM_CONTROL 0
#define BCM2835_PWM_STATUS  1
#define BCM2835_PWM_DMAC    2
#define BCM2835_PWM0_RANGE  4
#define BCM2835_PWM0_DATA   5
#define BCM2835_PWM_FIF1    6
#define BCM2835_PWM1_RANGE  8
#define BCM2835_PWM1_DATA   9

// Defines for PWM Clock, word offsets (ie 4 byte multiples)
#define BCM2835_PWMCLK_CNTL     40
#define BCM2835_PWMCLK_DIV      41
#define BCM2835_PWM_PASSWRD     (0x5A << 24)  ///< Password to enable setting PWM clock

#define BCM2835_PWM1_MS_MODE    0x8000  ///< Run in Mark/Space mode
#define BCM2835_PWM1_USEFIFO    0x2000  ///< Data from FIFO
#define BCM2835_PWM1_REVPOLAR   0x1000  ///< Reverse polarity
#define BCM2835_PWM1_OFFSTATE   0x0800  ///< Ouput Off state
#define BCM2835_PWM1_REPEATFF   0x0400  ///< Repeat last value if FIFO empty
#define BCM2835_PWM1_SERIAL     0x0200  ///< Run in serial mode
#define BCM2835_PWM1_ENABLE     0x0100  ///< Channel Enable

#define BCM2835_PWM0_MS_MODE    0x0080  ///< Run in Mark/Space mode
#define BCM2835_PWM_CLEAR_FIFO  0x0040  ///< Clear FIFO
#define BCM2835_PWM0_USEFIFO    0x0020  ///< Data from FIFO
#define BCM2835_PWM0_REVPOLAR   0x0010  ///< Reverse polarity
#define BCM2835_PWM0_OFFSTATE   0x0008  ///< Ouput Off state
#define BCM2835_PWM0_REPEATFF   0x0004  ///< Repeat last value if FIFO empty
#define BCM2835_PWM0_SERIAL     0x0002  ///< Run in serial mode
#define BCM2835_PWM0_ENABLE     0x0001  ///< Channel Enable

/// \brief bcm2835PWMClockDivider
/// Specifies the divider used to generate the PWM clock from the system clock.
/// Figures below give the divider, clock period and clock frequency.
/// Clock divided is based on nominal PWM base clock rate of 19.2MHz
/// The frequencies shown for each divider have been confirmed by measurement
typedef enum
{
    BCM2835_PWM_CLOCK_DIVIDER_32768 = 32768,   ///< 32768 = 585Hz
    BCM2835_PWM_CLOCK_DIVIDER_16384 = 16384,   ///< 16384 = 1171.8Hz
    BCM2835_PWM_CLOCK_DIVIDER_8192  = 8192,    ///< 8192 = 2.34375kHz
    BCM2835_PWM_CLOCK_DIVIDER_4096  = 4096,    ///< 4096 = 4.6875kHz
    BCM2835_PWM_CLOCK_DIVIDER_2048  = 2048,    ///< 2048 = 9.375kHz
    BCM2835_PWM_CLOCK_DIVIDER_1024  = 1024,    ///< 1024 = 18.75kHz
    BCM2835_PWM_CLOCK_DIVIDER_512   = 512,     ///< 512 = 37.5kHz
    BCM2835_PWM_CLOCK_DIVIDER_256   = 256,     ///< 256 = 75kHz
    BCM2835_PWM_CLOCK_DIVIDER_128   = 128,     ///< 128 = 150kHz
    BCM2835_PWM_CLOCK_DIVIDER_64    = 64,      ///< 64 = 300kHz
    BCM2835_PWM_CLOCK_DIVIDER_32    = 32,      ///< 32 = 600.0kHz
    BCM2835_PWM_CLOCK_DIVIDER_16    = 16,      ///< 16 = 1.2MHz
    BCM2835_PWM_CLOCK_DIVIDER_8     = 8,       ///< 8 = 2.4MHz
    BCM2835_PWM_CLOCK_DIVIDER_4     = 4,       ///< 4 = 4.8MHz
    BCM2835_PWM_CLOCK_DIVIDER_2     = 2,       ///< 2 = 9.6MHz, fastest you can get
    BCM2835_PWM_CLOCK_DIVIDER_1     = 1,       ///< 1 = 4.6875kHz, same as divider 4096
} bcm2835PWMClockDivider;

// Historical name compatibility
#ifndef BCM2835_NO_DELAY_COMPATIBILITY
#define delay(x) bcm2835_delay(x)
#define delayMicroseconds(x) bcm2835_delayMicroseconds(x)
#endif

#endif // BCM2835_H

/// @example blink.c
/// Blinks RPi GPIO pin 11 on and off

/// @example input.c
/// Reads the state of an RPi input pin

/// @example event.c
/// Shows how to use event detection on an input pin

/// @example spi.c
/// Shows how to use SPI interface to transfer a byte to and from an SPI device

/// @example spin.c
/// Shows how to use SPI interface to transfer a number of bytes to and from an SPI device

/// @example pwm.c
/// Shows how to use PWM to control GPIO pins

/// @example i2c.c
/// Command line utility for executing i2c commands with the 
/// Broadcom bcm2835. Contributed by Shahrooz Shahparnia.

/// example gpio.c
/// Command line utility for executing gpio commands with the 
///   Broadcom bcm2835. Contributed by Shahrooz Shahparnia.
