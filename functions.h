/*     functions.h
*/

/*                                     */
#define u8              uint8_t
//#define u8             unsigned char
//#define uint8_t        unsigned char


#define u16             uint16_t
//#define u16             unsigned short
//#define uint16_t        unsigned short

//#define int16_t		short

#define u32             uint32_t
//#define u32             unsigned int
//#define uint32_t        unsigned int

//#define int32_t		int

#define u64             uint64_t
//#define u64             unsigned long
//#define uint64_t        unsigned long

//#define TXBUFFER_SIZE    1024
//#define RXBUFFER_SIZE    1024
//#define TXBUFFER_SIZE    32
//#define RXBUFFER_SIZE    32
#define TXBUFFER_SIZE    64     // 64 = 0x40
#define RXBUFFER_SIZE    64
//#define TXBUFFER_SIZE    128     // 128 = 0x80
//#define RXBUFFER_SIZE    128
#define NUMB_VOUT        0x35     //  0 ... 0x11, 0x12, ... 0x33, 0x34



struct strct12  {
    u16 avr[12];
    u16 pk2pk[12];
    u16 dv[12];  };

struct strct12x32  {
    int32_t avr[12];
    int32_t pk2pk[12];
    int32_t dv[12];  };

struct offstrct12x32  {
    u16 vout[12];
    int32_t avr[12];
    int32_t pk2pk[12];
    int32_t dv[12];  };

struct ioffsetTable  {
    u16 convI;
    u16 vout;
    int16_t offset[12];
    u16 vout_avr[12];
};


/*     LEDs */
#define  Led1           GPIO_PIN_1
#define  Led2           GPIO_PIN_13
#define LedPort         GPIOE


/*      i2c             */

#define i2c1_sda        GPIO_PIN_9   //pb9
#define i2c1_scl        GPIO_PIN_9   //pa9

#define i2c2_sda        GPIO_PIN_11   //pb11
#define i2c2_scl        GPIO_PIN_13   //pb13

#define i2c3_sda        GPIO_PIN_1   //pc1
#define i2c3_scl        GPIO_PIN_7   //pa7


#define I2C_A2_C1       GPIO_PIN_8      //PD8
#define I2C_A2_C2       GPIO_PIN_7      //PD7
#define I2C_A2_C3       GPIO_PIN_5      //PD5



#define NISL_ADDR           0x60 // test        30
#define I2C_DUT1_VDD2       0xAA // VX21        50    i2cDut[1]
#define I2C_DUT2_VDD2       0xA8 // VX22        51    i2cDut[4]
#define I2C_DUT3_VDD2       0xA2 // VX23        54
#define I2C_DUT4_VDD2       0xA0 // VX24        55

#define I2C_DUT1_VDD1       0x8A // VX11        45      i2cDut[0] 
#define I2C_DUT2_VDD1       0x88 // VX12        41      i2cDut[3]
#define I2C_DUT3_VDD1       0xE2 // VX33        74
#define I2C_DUT4_VDD1       0xE0 // VX34        75

#define I2C_DUT1_VDDQ       0x82 // VX13        44    i2cDut[2]
#define I2C_DUT2_VDDQ       0x80 // VX14        40    i2cDut[5]
#define I2C_DUT3_VDDQ       0xEA // VX31        70
#define I2C_DUT4_VDDQ       0xE8 // VX32        71



#define I2C_VX11       0x8A // VX11        45      i2cDut[0] 
#define I2C_VX12       0x88 // VX12        41      i2cDut[3]
#define I2C_VX33       0xE2 // VX33        74
#define I2C_VX34       0xE0 // VX34        75

#define I2C_VX13       0x82 // VX13        44    i2cDut[2]
#define I2C_VX14       0x80 // VX14        40    i2cDut[5]
#define I2C_VX31       0xEA // VX31        70
#define I2C_VX32       0xE8 // VX32        71

#define I2C_VX21       0xAA // VX21        50    i2cDut[1]
#define I2C_VX22       0xA8 // VX22        51    i2cDut[4]
#define I2C_VX23       0xA2 // VX23        54
#define I2C_VX24       0xA0 // VX24        55


/*      SPI             */

#define SPI1_SCK        GPIO_PIN_1      // PA1
#define SPI1_MOSI       GPIO_PIN_5      // PB5
#define SPI1_MISO       GPIO_PIN_6      // PA6
#define SPI1_NSS        GPIO_PIN_0      // PB0

#define SPI2_SCK        GPIO_PIN_1      // PD1
#define SPI2_MOSI       GPIO_PIN_4      // PD4
#define SPI2_MISO       GPIO_PIN_3      // PD3

#define ADC1_NSS        GPIO_PIN_2      // PA2
#define ADC2_NSS        GPIO_PIN_6      // PC6

#define RST_ADC1        GPIO_PIN_9      // pd9
#define RST_ADC2        GPIO_PIN_9      // pc9

#define START_ADC1      GPIO_PIN_4      // pa4
#define START_ADC2      GPIO_PIN_8      // pc8


//	AD5592	u12, u13
#define DAC1_NSS        GPIO_PIN_14		// PB14	Group B		U12		SPI1_DAC1_CS#
#define DAC2_NSS        GPIO_PIN_15		// PB15	Group B		U13		SPI1_DAC2_CS#

#define DAC00_RST        GPIO_PIN_7     // PE7 
#define DAC23_RST        GPIO_PIN_4     // PE4
#define DAC12_RST        GPIO_PIN_3     // PE3 

/*
PAGE 42 TO 49: THE FIRST INSTANCE OF VESUVIUS BLOCK (VSV_8XDUT_BLK - PAGE3_I2)
	PAGE 50, 51, 52, 53: FOUR INSTANCES OF THE VSV_BLK USED IN THE 1ST INSTANCE OF VESUVUS BLOCK
	U52, U54, 56, 58

PAGE 54 TO 61: THE SECOND INSTANCE OF VSV_8XDUT_BLK (PAGE3_I3)
	PAGE 62, 63, 64, 65: FOUR INSTANCES OF THE VSV_BLK USED IN THE 2ND INSTANCE OF VESUVUS BLOCK

PAGE 66 TO 73: THE 3RD INSTANCE OF VESUVUS BLOCK (VSV_8XDUT_BLK - PAGE4_I2)
	PAGE 74, 75, 76, 77: FOUR INSTANCES OF THE VSV_BLK USED IN THE 3RD INSTANCE OF VSV_8XDUT_BLK BLOCK

PAGE 80 TO 87: THE 4TH INSTANCE OF VSV_8XDUT_BLK
	PAGE 53, 54, 55, 56: FOUR INSTANCES OF THE VSV_BLK USED IN THE 4TH INSTANCE OF VSV_8XDUT_BLK BLOCK
*/


//	AD7124-8:
#define ADC1_7124_NSS        GPIO_PIN_0		// PB0	Group C		SPI1_ADC_CS#	page 3, I3;		page 48, U24
#define ADC2_7124_NSS        GPIO_PIN_7		// PB7	Group D		SPI2_ADC_CS#	page 3, I2;		page 86, U15
#define ADC3_7124_NSS        GPIO_PIN_10		// PB10	Group B		SPI3_ADC_CS#	page 4, I2;		page 60, U33
#define ADC4_7124_NSS        GPIO_PIN_12		// PB12	Group A		SPI4_ADC_CS#	page 4, I3;		page 72, U42



/*      export functions    */
void Init();
 
void Test();
void DAC_Init();
void ADC_Init();
void set_LPDDR4();
void Reset_ADC();
void SetVout_3026(uint16_t value, uint8_t id);
void SetVout_4644(uint16_t value, uint8_t id);
int32_t ISL_Read_Vshunt(u8 id, u8 scale);
u16 ISL_Read_Vaux(u8 id)  ;
void ISL_Test_0(void);
void XO2_Test(void);

//v     vvv

#define VREFM	 600	//  LTM4671	VREF = 600mV

// For DAC IO0	VDDQ_IN = 2V to 2.5V	sch page 42: R10M=R105, R30M=R106		from BOM:	R105=24K9, R106=75k
// For DAC IO1	VDD_IN = 2V to 2.5V		sch page 42: R10M=R101, R30M=R100		from BOM:	R101=24K9, R100=75k
#define R10DD	24900	//  LTM4671	R10 = 24K9
#define R20DD	60400	//  LTM4671	R20 = 60.4K
#define R30DD	75000	//  LTM4671	R30 = 75K

// For DAC IO2	VPP_IN = 2.1V to 3V		sch page 42: R10M=R102, R30M=R103		from BOM:
#define R10PP	24300	//  LTM4671	R10 = 24K3
#define R20PP	60400	//  LTM4671	R20 = 60.4K
#define R30PP	37400	//  LTM4671	R30 = 37K4

#define VREFC	 400	//  LTC3026	VREF = 400mV
#define R10C	4750	//  LTC3026	R10 = 4.75K
#define R20C	10700	//  LTC3026	R20 = 10.7K
#define R30C	4420	//  LTC3026	R30 = 4.42K

#define VREF_DAC	2500	//  AD559xR	VREF DAC = 2500mV
#define DAC_RESOL	4096	//  AD559xR	DAC resolution (12-bit) = 4096


/*

u16 DACaddress[4][3] = {        0xc000,0xa000,0xe000,   //dut1 VDD1 VDD2 VDDQ 
                                0xd000,0xb000,0xf000,   //dut2
                                0xe000,0xa000,0xc000,   //dut3
                                0xf000,0xb000,0xd000    //dut4
                        };


u16 DACVout[8]={0x8000,0x9000,0xa000,0xb000,0xc000,0xd000,0xe000,0xf000};


u16  ADCaddress[4][3] = { 0x0002,0x0006,0x0004,   //dut1   VDD1 VDD2 VDDQ
                          0x0003,0x0007,0x0005,    //dut2
                          0x0006,0x0002,0x0004,    //dut3
                          0x0007,0x0003,0x0005      //dut4
                         };

//u8 i2cDut[13] ={NISL_ADDR,I2C_DUT1_VDD1,I2C_DUT1_VDD2,I2C_DUT1_VDDQ,
u8 i2cDut[12] ={I2C_DUT1_VDD1,I2C_DUT1_VDD2,I2C_DUT1_VDDQ,
                I2C_DUT2_VDD1,I2C_DUT2_VDD2,I2C_DUT2_VDDQ,
                I2C_DUT3_VDD1,I2C_DUT3_VDD2,I2C_DUT3_VDDQ,
                I2C_DUT4_VDD1,I2C_DUT4_VDD2,I2C_DUT4_VDDQ,
            } ;         

//              0       1               2               3               4         5  6  7  8  9
u8 i2cVxy[35] ={
                0,      0,              0,              0,              0,        0, 0, 0, 0, 0,
                0,      I2C_VX11,       I2C_VX12,       I2C_VX13,       I2C_VX14, 0, 0, 0, 0, 0,
                0,      I2C_VX21,       I2C_VX22,       I2C_VX23,       I2C_VX24, 0, 0, 0, 0, 0,
                0,      I2C_VX31,       I2C_VX32,       I2C_VX33,       I2C_VX34
               } ;         

//              0       1               2               3               4         5  6  7  8  9  A  B  C  D  E  F
u8 i2cHVxy[0x35] = {
                0,      0,              0,              0,              0,        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0,      I2C_VX11,       I2C_VX12,       I2C_VX13,       I2C_VX14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0,      I2C_VX21,       I2C_VX22,       I2C_VX23,       I2C_VX24, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0,      I2C_VX31,       I2C_VX32,       I2C_VX33,       I2C_VX34
               } ;         

u8 i2c_Vxy[12] ={ I2C_VX11, I2C_VX12, I2C_VX13, I2C_VX14,
                  I2C_VX21, I2C_VX22, I2C_VX23, I2C_VX24,
                  I2C_VX31, I2C_VX32, I2C_VX33, I2C_VX34
               } ;         

u8 i2xy[12] ={ 0x11, 0x12, 0x13, 0x14,
               0x21, 0x22, 0x23, 0x24,
               0x31, 0x32, 0x33, 0x34
             } ;         

//              0       1       2       3       4       5       6       7       
u8 xy2i[53] = {
                0x55,   0x55,   0x55,   0x55,   0x55,   0x55,   0x55,   0x55,
                0x55,   0x55,   0x55,   0x55,   0x55,   0x55,   0x55,   0x55,
                0x55,   0,      1,      2,      3,      0x55,   0x55,   0x55,   //      1x
                0x55,   0x55,   0x55,   0x55,   0x55,   0x55,   0x55,   0x55,
                0x55,   4,      5,      6,      7,      0x55,   0x55,   0x55,   //      2x
                0x55,   0x55,   0x55,   0x55,   0x55,   0x55,   0x55,   0x55,
                0x55,   8,      9,      10,     11                              //      3x
              } ;         

*/
/*
//              0       1       2       3       4       5       6       7       8       9
u8 xy2i[35] = {
                0x55,   0x55,   0x55,   0x55,   0x55,   0x55,   0x55,   0x55,   0x55,   0x55,
                0x55,   0,      1,      2,      3,      0x55,   0x55,   0x55,   0x55,   0x55,   // decimal 11, 12, etc.
                0x55,   4,      5,      6,      7,      0x55,   0x55,   0x55,   0x55,   0x55,
                0x55,   8,      9,      10,     11
              } ;         
*/

