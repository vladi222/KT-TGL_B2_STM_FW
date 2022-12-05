/*	I:\F_\TrueSTUDIO_DA_202\DA_2_0_2\src
 *      #########################################################################
 *              functions  for   commands                for 4DA 1.9, IAR 8.30
 *      #########################################################################
*/
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "functions.h"
//#include "Xo2.h"
 
#define FLASH_USER_START_ADDR   ((uint32_t)0x0800C000)   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ((uint32_t)0x0801FFFF)   /* End @ of user Flash area */
#define FLASH_TYPEERASE_PAGES     ((uint32_t)0x00)  
#define PageSize      2048                                           
/*extern variables  */
/*
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
*/
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;

extern void BSP_LED_On(uint16_t pin);
extern void BSP_LED_Off(uint16_t pin);
extern void BSP_LED_Toggle(uint16_t pin);

#define OK 0

//uint8_t rxBffer[128];
//uint8_t txBffer[128];
//      Definitions in main():
//extern uint8_t txBuffer[TXBUFFER_SIZE];
//extern uint8_t rxBuffer[RXBUFFER_SIZE];

uint32_t FWversion;

uint8_t rxBffer[TXBUFFER_SIZE];
uint8_t txBffer[TXBUFFER_SIZE];

//u32 I2C_Comm(I2C_HandleTypeDef* I2Cx, u8 * pIn, u16 NumByteToWrite, u8 * pOut, u16 NumByteToRead, u8 I2C_ADDRESS);

u8 data[4];
float data2;
u32 statusf;

u32 inst_I_80[12] = {0};
u32 inst_I_200[12] = {0};
u32 absI_Max_80[12] = {0};
u32 absI_Max_200[12] = {0};
u32 abs_count;

//                              0       1       2       3       4       5       6       7       8       9       a       b       c       d       e       f
//                              1       2       4       8       16      32      64      128     256     512     1024    2048    4096    4096    4096    4096
u16 time_x_samples[8][16] =  {
                            {   1,      1,      1,      1,      1,      2,      4,      8,      16,     33,     66,     131,    262,    262,    262,    262     },       // 64us         000     
                            {   1,      1,      1,      1,      2,      4,      8,      16,     33,     66,     131,    262,    524,    512,    512,    524     },       // 128us        001
                            {   1,      1,      1,      2,      4,      8,      16,     33,     66,     131,    262,    524,    1048,   1048,   1048,   1048    },       // 256us        010
                            {   1,      1,      2,      4,      8,      16,     33,     66,     131,    262,    524,    1048,   2097,   2097,   2097,   2097    },       // 512us        011
                            {   1,      2,      4,      8,      16,     33,     66,     131,    262,    524,    1048,   2097,   4194,   4194,   4194,   4194    },       // 1024us       10x
                            {   1,      2,      4,      8,      16,     33,     66,     131,    262,    524,    1048,   2097,   4194,   4194,   4194,   4194    },       // 1024us       10x
                            {   2,      4,      8,      16,     33,     66,     131,    262,    524,    1048,   2097,   4194,   8389,   8389,   8389,   8389    },       // 2048us       11x
                            {   2,      4,      8,      16,     33,     66,     131,    262,    524,    1048,   2097,   4194,   8389,   8389,   8389,   8389    }        // 2048us       11x
                        };


u16 DACaddress[4][3] = {	{ 0xc000,0xa000,0xe000},   //dut1 VDD1 VDD2 VDDQ
							{ 0xd000,0xb000,0xf000},   //dut2
							{ 0xe000,0xa000,0xc000},   //dut3
							{ 0xf000,0xb000,0xd000}    //dut4
                        };

/*
u16  ADCaddress[4][3] = { 0x0002,0x0006,0x0004,   //dut1   VDD1 VDD2 VDDQ
                          0x0003,0x0007,0x0005,    //dut2
                          0x0006,0x0002,0x0004,    //dut3
                          0x0007,0x0003,0x0005      //dut4
                         };
*/

u16 DACVout[8]={0x8000,0x9000,0xa000,0xb000,0xc000,0xd000,0xe000,0xf000};

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
                0,      0,              0,              0,              0,        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,      // 0x0y
                0,      I2C_VX11,       I2C_VX12,       I2C_VX13,       I2C_VX14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,      // 0x1y
                0,      I2C_VX21,       I2C_VX22,       I2C_VX23,       I2C_VX24, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,      // 0x2y
                0,      I2C_VX31,       I2C_VX32,       I2C_VX33,       I2C_VX34                                        // 0x3y
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


/*
//              0       1       2       3       4       5       6       7       8       9
u8 xy2i[35] = {
                0x55,   0x55,   0x55,   0x55,   0x55,   0x55,   0x55,   0x55,   0x55,   0x55,
                0x55,   0,      1,      2,      3,      0x55,   0x55,   0x55,   0x55,   0x55,   // decimal 11, 12, etc.
                0x55,   4,      5,      6,      7,      0x55,   0x55,   0x55,   0x55,   0x55,
                0x55,   8,      9,      10,     11
              } ;         
*/



// XO2 I2C address:
//      0x82 for programming
//      0x84 for user
#define I2C00_LBMachXo2 0x80 //MachXo2_2; //0x80; 
#define I2C0_LBMachXo2 0x82 //MachXo2_2; //0x82; 
#define I2C_LBMachXo2 0x84 //MachXo2_2; //0x84; 



struct strct12          isl_V1, isl_V2;
struct strct12x32       isl_I1, isl_I2, isl_I21;
struct offstrct12x32    ioff_v0, ioff_v1, ioff_v2, ioff_v3, ioff_v4, ioff_v5, ioff_v6, ioff_v7;
struct ioffsetTable     ioffTab_v0, ioffTab_v1, ioffTab_v2, ioffTab_v3, ioffTab_v4, ioffTab_v5, ioffTab_v6, ioffTab_v7;


void fpga_reset(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // MCU PA0/WKUP
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // MCU PA0/WKUP
//	HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // MCU PA0/WKUP
}

/**
  * @}
  */ 
void BSP_LED_Init()
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the GPIO_LED Clock */
 // LEDx_GPIO_CLK_ENABLE(Led);
  __GPIOE_CLK_ENABLE();
  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = Led1|Led2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_Init(LedPort, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(LedPort, Led1, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(LedPort, Led2, GPIO_PIN_RESET); 
}

/**
  * @brief  Turns selected LED On
  */
void BSP_LED_On(uint16_t pin)
{
  HAL_GPIO_WritePin(LedPort, pin, GPIO_PIN_SET); 
}

/**
  * @brief  Turns selected LED Off.
  */
void BSP_LED_Off(uint16_t pin)
{
  HAL_GPIO_WritePin(LedPort, pin, GPIO_PIN_RESET); 
}

/**
  * @brief  Toggles the selected LED.
*/ 
void BSP_LED_Toggle(uint16_t pin)
{
  HAL_GPIO_TogglePin(LedPort, pin);
}


/*  XO2  Init  */
void Xo2_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  //PE9         IRQ_MCU         External Interrupt Mode with Falling edge trigger detection
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  //PE8         MCU_CMD         Output Push Pull Mode
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  //PE11,15     vectors, in/out         Output Open Drain Mode
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  //PC4     vector, in/out         Output Open Drain Mode
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  //PD13     vector, in/out         Output Open Drain Mode
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  //PD15        RUN_LTM         Output Push Pull Mode   R297, pull-up to 12V, not populated  
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  //pins to 1
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); 
  HAL_Delay(100);

  // reset I2C XO2 -- MCU commnd, vector(0,0,0,0)
  // vector(0,0,0,0) - PE11, PE15, PC4, PD13
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_Delay(200);
  // MCU commnd: PE8 0-1
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET); 
  HAL_Delay(200);

  // vector(1,1,1,1) - PE11, PE15, PC4, PD13    end of XO2 I2C reset
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
}

/*      FPGA (U35) to MCU (U7):
LOCATE COMP "vectr[3]" SITE "29" ;# to MCU, PE11        U7.M9
IOBUF PORT "vectr[3]" PULLMODE=UP IO_TYPE=LVCMOS33 SLEWRATE=FAST ;
#
LOCATE COMP "vectr[2]" SITE "30" ;# to MCU, PE15        U7.M12
IOBUF PORT "vectr[2]" PULLMODE=UP IO_TYPE=LVCMOS33 SLEWRATE=FAST ;
#
LOCATE COMP "vectr[1]" SITE "47" ;# fpb14b and MCU PC4-ADC13 and Power Good for 3.3V (no use for ADC)   U7.K5
IOBUF PORT "vectr[1]" PULLMODE=UP IO_TYPE=LVCMOS33 ;
#
LOCATE COMP "vectr[0]" SITE "48" ;# from MCU, PD13 - SPI2_NSS           U7.H12
IOBUF PORT "vectr[0]" PULLMODE=UP IO_TYPE=LVCMOS33 SLEWRATE=FAST ;

vectr[3:0]:     PE11    PE15    PC4     PD13


MCU Setting the FPGA RALERT Registers:
			if (MEM_ADDR = (REG_ADDR + x"61"))	then	-- REG_ADDR + x"61" = 0xE1
				ralert11 <= MEM_WD(3 downto 0);
				ralert12 <= MEM_WD(7 downto 4);
			elsif (MEM_ADDR = (REG_ADDR + x"62"))	then
				ralert21 <= MEM_WD(3 downto 0);
				ralert22 <= MEM_WD(7 downto 4);
			elsif (MEM_ADDR = (REG_ADDR + x"63"))	then
				ralert31 <= MEM_WD(3 downto 0);
				ralert32 <= MEM_WD(7 downto 4);
*/


  //PE8         MCU_CMD         Output Push Pull Mode
  //PE9         IRQ_MCU         External Interrupt Mode with Falling edge trigger detection
  // vectors - PE11, PE15, PC4, PD13
void ReadVectorPins(GPIO_PinState* vect_pin)            // vect_pin[5]
{
  vect_pin[5] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8);    //      MCU_CMD
  vect_pin[4] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);    //      IRQ_MCU
  vect_pin[3] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);   //      vector
  vect_pin[2] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15);   //      vector
  vect_pin[1] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);    //      vector
  vect_pin[0] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);   //      vector
}




u8 txBuffer[TXBUFFER_SIZE];
u8 rxBuffer[RXBUFFER_SIZE];
//extern uint8_t txBuffer[TXBUFFER_SIZE];
//extern uint8_t rxBuffer[RXBUFFER_SIZE];
u16 nIn=0;
u32 status=0;         // declared in main()
u32 statusWord;

u8 reg12or60V[NUMB_VOUT] = {0x55};      // #define NUMB_VOUT        0x35     //  0 ... 0x11 ... 0x33, 0x34 (in functions.h)


                

u8 rxBuff[8]={0};
int16_t rrvshunt;
int32_t rvshunt;
u32 r_addr_scale_id;
int int_id;



//      #########################################################
//      #################       new     vvv     #################
//      #########################################################


int icount;

u16 V_isl[12];
float tavr[12];

            

u8 thcode(u16 value)
{
 u8 rdata;
 
        //    D[15:10]  = 0000 00
        //    D[9]      = IOUT_ DIR     0: VINP to VINM         1: VINM to VINP
        //    D[8:7]    = 0 0
        //    D[6]      = Range     0: 80mV         1: 40mV
        //    D[5:0]    = VSHUNT_OC_SET         6-bit threshold value   from 25% to 125% of the full-scale range - resolution: 1.56%    (100/64)
        //                                            80mV range - threshold range 20mV...100mV - resolution = 1.25mV
        //                                            40mV range - threshold range 10mV...50mV  - resolution = 625uV
  if (value > 100)      rdata = 0x3F;    //  D[6] = 0;     D[5:0] = 0b 11 1111;
  else
  {
    if (value < 10)     rdata = 0x40;    //  D[6] = 1;     D[5:0] = 0b 00 0000;
    else
    {
      if (value < 50)
      {
//        data2 = ((float) (value - 10)) * ((float) 0x3F) / ((float) 50);  // for 50mV: 3F; for 10mV: 0
        data2 = (float) ((value - 10) * 0x3F / 50);  // for 50mV: 3FF; for 10mV: 0
//        data[2] = ((u8) data2) && 0x3F;   // just in case
        rdata = 0x40 + (u8) data2;    //  D[6] = 1;
      }
      else        // value >= 50
      {
        data2 = ((float) (value - 20)) * ((float) 0x3F) / ((float) 100);  // for 100mV: 3F; for 20mV: 0
//        data[2] = ((u8) data2) && 0x3F;    //  D[6] = 0;
        rdata = (u8) data2;    //  D[6] = 0;
      }
    }
  }
  
  return (rdata);
}




u32 Xo2_read_str(u8 regAddr, u8 * read_str)     // reads 8 bytes from Xo2 starting from regAddr
{
  u8  cmd[2];

  cmd[0] = 0x0B;        // 0x0B - read memory - 8 bytes  
  cmd[1] = regAddr;  
//  status = I2C_ISL(&hi2c2, cmd, 2, &read_str[0], 8, I2C_LBMachXo2);

  return status;
}


u16 VauxTab[12];
u16 VoutTab[12];
u32 bstatTab[12];
int32_t Vshunt1Tab[12];
int32_t Vshunt2Tab[12];
u8 portTab[12];
u8 DPM_Tab[12];
u8 alertE1[6];
u8 force91[6];
u8 maskA1[6];
u8 rshdnD1[3];
GPIO_PinState  vect_pin[6];




/*	AD5592	defined in functions.h:
		u12, u13
#define DAC1_NSS        GPIO_PIN_14		// PB14	Group B		SPI1_DAC1_CS#
#define DAC2_NSS        GPIO_PIN_15		// PB15	Group B		SPI1_DAC2_CS#
*/


void DAC_NSS_test(int id)
{
	  u16 nss_pin[2] = {DAC1_NSS, DAC2_NSS,};       // u12, u13
  u16 time = 1000;

	HAL_GPIO_WritePin(GPIOB, nss_pin[id], GPIO_PIN_RESET);

	for(time=0; time<5; time++) {};

	HAL_GPIO_WritePin(GPIOB, nss_pin[id], GPIO_PIN_SET);

	for(time=0; time<10; time++) {};


	HAL_GPIO_WritePin(GPIOB, nss_pin[id], GPIO_PIN_RESET);

	for(time=0; time<5; time++) {};

	HAL_GPIO_WritePin(GPIOB, nss_pin[id], GPIO_PIN_SET);

	for(time=0; time<10; time++) {};
}


 HAL_StatusTypeDef hstatus;

HAL_StatusTypeDef DAC_WriteReg_16SPI(u16 value, int id)	// for 16-bit SPI
{
	  u16 nss_pin[2] = {DAC1_NSS, DAC2_NSS,};       // u12, u13
  u16 time = 1000;

	HAL_GPIO_WritePin(GPIOB, nss_pin[id], GPIO_PIN_RESET);

	hstatus = HAL_SPI_Transmit( &hspi1, (uint8_t * )&value, 1, time);
// 221115	hstatus = HAL_SPI_Transmit( &hspi1, (uint8_t * )value, 1, time);

	HAL_GPIO_WritePin(GPIOB, nss_pin[id], GPIO_PIN_SET);
  
	for(time=0; time<20; time++) {};

	return hstatus;
}
/*
HAL_StatusTypeDef DAC_WriteReg(u16 value, int id)	// for 8-bit SPI
{
	  u16 nss_pin[4] = {DAC1_NSS, DAC2_NSS, DAC3_NSS, DAC4_NSS};       // u18, u9, u27, u36
  u16 time = 1000;
  u8 value8_l, value8_h;


//	value8_l = 0;
//	value8_h = 0;

	value8_l = (u8) value;
	value8_h = (u8) (value >> 8);

	HAL_GPIO_WritePin(GPIOB, nss_pin[id], GPIO_PIN_RESET);

	if (id == 0)  {
		hstatus = HAL_SPI_Transmit( &hspi1, (u8 * )&value8_h, 1, time);
		hstatus = HAL_SPI_Transmit( &hspi1, (u8 * )&value8_l, 1, time);
	}
	else	if (id == 1)  {
		hstatus = HAL_SPI_Transmit( &hspi2, (u8 * )&value8_h, 1, time);
		hstatus = HAL_SPI_Transmit( &hspi2, (u8 * )&value8_l, 1, time);
	}
	else	if (id == 2)  {
		hstatus = HAL_SPI_Transmit( &hspi3, (u8 * )&value8_h, 1, time);
		hstatus = HAL_SPI_Transmit( &hspi3, (u8 * )&value8_l, 1, time);
	}
	else	if (id == 3)  {
		hstatus = HAL_SPI_Transmit( &hspi4, (u8 * )&value8_h, 1, time);
		hstatus = HAL_SPI_Transmit( &hspi4, (u8 * )&value8_l, 1, time);
	}

	HAL_GPIO_WritePin(GPIOB, nss_pin[id], GPIO_PIN_SET);

	HAL_Delay(1);		// or	for(time=0; time<20; time++) {};

	return hstatus;
}
*/


/**
  * @brief  Transmit and Receive an amount of data in blocking mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @param  Size amount of data to be sent and received
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
//HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
//                                          uint32_t Timeout)

//HAL_StatusTypeDef DAC_WriteReg_16SPI(u16 value, int id)	// for 16-bit SPI
//	hstatus = HAL_SPI_Transmit( &hspi1, (uint8_t * )&value, 1, time);

//HAL_StatusTypeDef DAC_SPI_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, int id)
HAL_StatusTypeDef DAC_SPI_TransmitReceive(uint16_t pTxData, uint8_t *pRxData, int id)
{
	  u16 nss_pin[2] = {DAC1_NSS, DAC2_NSS,};       // u12, u13
u16 time = 1000;

	HAL_GPIO_WritePin(GPIOB, nss_pin[id], GPIO_PIN_RESET);

//	hstatus = HAL_SPI_Transmit( &hspi1, (uint8_t * )&value, 1, time);
	hstatus = HAL_SPI_TransmitReceive( &hspi1, (uint8_t * )&pTxData, pRxData, 2, 1);

	HAL_GPIO_WritePin(GPIOB, nss_pin[id], GPIO_PIN_SET);

// HAL_Delay(1);
for(time=0; time<20; time++) {};
return hstatus;
}


//HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)

HAL_StatusTypeDef DAC_ReadReg_16SPI(u16 * value, int id)
{
	  u16 nss_pin[2] = {DAC1_NSS, DAC2_NSS,};       // u12, u13
  u16 time = 1000;
  uint32_t a;

  a = *value;

	HAL_GPIO_WritePin(GPIOB, nss_pin[id], GPIO_PIN_RESET);

//  HAL_SPI_Receive( &hspi2, (u8 * )value, 1, time);
	hstatus = HAL_SPI_Receive( &hspi1, value, 1, time);

	HAL_GPIO_WritePin(GPIOB, nss_pin[id], GPIO_PIN_SET);

// HAL_Delay(1);
  for(time=0; time<20; time++) {};
  return hstatus;
}


//	HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)

//HAL_StatusTypeDef DAC_ReadReg(u16 * value, int id)
/*
u16 DAC_ReadReg(int id)
{
	 HAL_StatusTypeDef hstatus_DAC;
  u16 nss_pin[4] = {DAC1_NSS, DAC2_NSS, DAC3_NSS, DAC4_NSS};       // u18, u9, u27, u36
  u16 time = 1000;
//  u16 value8_l, value8_h;
  u16 value16;
  u8 value8_l, value8_h;

//  	value8_l = 0x5A;
//  	value8_h = 0x3C;

	HAL_GPIO_WritePin(GPIOB, nss_pin[id], GPIO_PIN_RESET);

//  HAL_SPI_Receive( &hspi2, (u8 * )value, 1, time);
	if (id == 0)	{
		hstatus_DAC = HAL_SPI_Receive( &hspi1, &value8_h, 1, time);
		hstatus_DAC = HAL_SPI_Receive( &hspi1, &value8_l, 1, time);
	}
	if (id == 3)	{
		hstatus = HAL_SPI_Receive( &hspi4, &value8_h, 1, time);
		hstatus = HAL_SPI_Receive( &hspi4, &value8_l, 1, time);
	}

	HAL_GPIO_WritePin(GPIOB, nss_pin[id], GPIO_PIN_SET);

	value16 = ( ((u16) value8_h) << 8) + (u16) value8_l;
//	value16 = value8_h;
//	value16 = (value16 << 8) + (u16) value8_l;
//	value16 = 256 * value8_h + (u16) value8_l;

//	*value = value16;

//	HAL_Delay(1);
	for(time=0; time<20; time++) {};
//	return hstatus;
	return value16;
}
*/

float DAC_Code(float vout, float vref, float r10, float r20, float r30, float resolution, float dac_vref)
{
  float code;
//  float p0, p1, p2;
  
  code=(vref+vref*r30*(1/r10+1/r20)-vout*r30/r20)*resolution/dac_vref;
//  p0=1/r10+1/r20;
//  p1=vref+vref*r30*p0;
//  p2=vout*r30/r20;
//  code=(p1-p2)*resolution/dac_vref;
  return code;
}





//HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
/* From		stm32f4xx_hal_spi.c
@note The max SPI frequency depend on SPI data size (8bits, 16bits),
      SPI mode(2 Lines fullduplex, 2 lines RxOnly, 1 line TX/RX) and Process mode (Polling, IT, DMA).
@note
     (#) TX/RX processes are HAL_SPI_TransmitReceive(), HAL_SPI_TransmitReceive_IT() and HAL_SPI_TransmitReceive_DMA()
     (#) RX processes are HAL_SPI_Receive(), HAL_SPI_Receive_IT() and HAL_SPI_Receive_DMA()
     (#) TX processes are HAL_SPI_Transmit(), HAL_SPI_Transmit_IT() and HAL_SPI_Transmit_DMA()
*/




/*   reset ADC devices   */
void Reset_ADC(void)
{
    //RST_ADC1   GPIO_PIN_9 //pd9
    // RST_ADC2   GPIO_PIN_9 //pc9
   HAL_GPIO_WritePin(GPIOD,RST_ADC1,GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOC,RST_ADC2,GPIO_PIN_RESET);
   HAL_Delay(1);
   HAL_GPIO_WritePin(GPIOD,RST_ADC1,GPIO_PIN_SET);
   HAL_GPIO_WritePin(GPIOC,RST_ADC2,GPIO_PIN_SET);   
}

/*reset DAC devices   */
void Reset_DAC(void)
{
// DAC00_RST       GPIO_PIN_7//PE7 
// DAC23_RST       GPIO_PIN_4//PE4
// DAC12_RST       GPIO_PIN_3//PE3 

   HAL_GPIO_WritePin(GPIOE,DAC00_RST,GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOE,DAC12_RST,GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOE,DAC23_RST,GPIO_PIN_RESET);
   HAL_Delay(1);
   HAL_GPIO_WritePin(GPIOE,DAC00_RST,GPIO_PIN_SET);
   HAL_GPIO_WritePin(GPIOE,DAC12_RST,GPIO_PIN_SET);  
   HAL_GPIO_WritePin(GPIOE,DAC23_RST,GPIO_PIN_SET);
   HAL_Delay(1);
   
//v   DAC_Init();
   
}

//v     vvv

void SetVIN_DD	(u16 vout, u8 id)		// id for DAC dev.	0 to 1
{
  float code;
  u16	value;
  u8 dac_io;

//	  dac_io = 0;	// for DAC IO0		U27	U30		DAC_DD2_1 - R186;	DAC_DD2_3 - R205
//	  dac_io = 1;	// for DAC IO1		U28	U31		DAC_DD2_2 - R188;	DAC_DD2_4 - R207			Bottom
//	  dac_io = 2;	// for DAC IO2		U26	U29		DAC_IN12 - R176;	DAC_IN34 - R195
  	  dac_io = 4;	// for DAC IO3		U22	U23		DAC_DD1 - R154;		DAC_DDQ - R157 (not pop)	Bottom

//		LTM4671 Vout0/3 acceptable range:		0.6V to 3.3V
//	if (vout < 600)		vout = 600;
//	if (vout > 3300)	vout = 3300;
//	VDD_IN range:	2V to 2.5V
//	if (vout < 2000)	vout = 2000;
	if (vout > 2500)	vout = 2500;

//	code = (VREFM + VREFM * R30M * (1 / R10M + 1 / R20M) � vout * R30M / R20M) * DAC_RESOL / VREF_DAC;
	code = DAC_Code((float)vout, VREFM, R10DD, R20DD, R30DD, DAC_RESOL, VREF_DAC);

	if (code < 0)		code = 0;
	if (code > 0xFFF)	code = 0xFFF;

	value = 0x8000 | (dac_io*0x1000) | (u16)code;
	DAC_WriteReg_16SPI(value, id);        //      u5:     dac 'id', data[11:0]
}



void SetVDD_4671(u16 vout, u8 id)		// id for DAC dev.	0 to 3	Group C, D, B, A
{
  float code;
  u16	value;
  u8 dac_io;

  	  dac_io = 0;	// for DAC IO0

//		LTM4671 Vout0/3 acceptable range:		0.6V to 3.3V
//	if (vout < 600)		vout = 600;
//	if (vout > 3300)	vout = 3300;
//	VDD_IN range:	2V to 2.5V
//	if (vout < 2000)	vout = 2000;
	if (vout > 2500)	vout = 2500;

//	code = (VREFM + VREFM * R30M * (1 / R10M + 1 / R20M) � vout * R30M / R20M) * DAC_RESOL / VREF_DAC;
	code = DAC_Code((float)vout, VREFM, R10DD, R20DD, R30DD, DAC_RESOL, VREF_DAC);

	if (code < 0)		code = 0;
	if (code > 0xFFF)	code = 0xFFF;

	value = 0x8000 | (dac_io*0x1000) | (u16)code;
	DAC_WriteReg(value, id);        //      u5:     dac 'id', data[11:0]
}

void SetVDDQ_4671(u16 vout, u8 id)		// id for DAC dev.	0 to 3	Group C, D, B, A
{
  float code;
  u16	value;
  u8 dac_io;

  	  dac_io = 1;	// for DAC IO1

//		LTM4671 Vout0/3 acceptable range:		0.6V to 3.3V
//	VDDQ_IN range:	2V to 2.5V
//	if (vout < 2000)	vout = 2000;
	if (vout > 2500)	vout = 2500;

	code = DAC_Code((float)vout, VREFM, R10DD, R20DD, R30DD, DAC_RESOL, VREF_DAC);

	if (code < 0)		code = 0;
	if (code > 0xFFF)	code = 0xFFF;

	value = 0x8000 | (dac_io*0x1000) | (u16)code;
	DAC_WriteReg(value, id);        //      dac 'id', data[11:0]
}

void SetVPP_4671(u16 vout, u8 id)		// id for DAC dev.	0 to 3	Group C, D, B, A
{
  float code;
  u16	value;
  u8 dac_io;

  	  dac_io = 2;	// for DAC IO2

//		LTM4671 Vout1/2 acceptable range:		0.6V to 5.5V
//	if (vout < 600)		vout = 600;
//	if (vout > 5500)	vout = 5500;
//	VPP_IN range:	2.1V to 3V
//	if (vout < 2100)	vout = 2100;
	if (vout > 3000)	vout = 3000;

//	code = (VREFM + VREFM * R30M * (1 / R10M + 1 / R20M) � vout * R30M / R20M) * DAC_RESOL / VREF_DAC;
	code = DAC_Code((float)vout, VREFM, R10PP, R20PP, R30PP, DAC_RESOL, VREF_DAC);

	if (code < 0)		code = 0;
	if (code > 0xFFF)	code = 0xFFF;

	value = 0x8000 | (dac_io*0x1000) | (u16)code;
	DAC_WriteReg(value, id);        //      dac 'id', data[11:0]
}



u8 deltaT;
int convT, convS;

//      current offset for all ports
void ioffset(u16 vout, u16 count, u8 scale, struct offstrct12x32 * isl, struct ioffsetTable * ioffT)        // count: # of samples for stats; dt: delay between iterations (giving time for a new measurement); scale = 1,2
{
 u8 xy;
 u8 data[4];
 u8 buff[4];
// int convT, convS;

  data[0] = 0xD4;  // config I channels R/W
//  status = I2C_ISL(&hi2c3, data, 1, buff, 2, i2cHVxy[0x21]);  // read config I channels reg for port 0x21 -- all ports are supposed to have the same config
  convT = buff[1] & 0x07;
  convS = ((buff[1] & 0x78) >> 3);
//  ioffT->convI = (buff[1] & 0x07) + ((buff[1] & 0x78) << 1);      // config I/V ch in 0x24 command format -- nibble 1 = # of samples; nibble 2 = conv time
  ioffT->convI = (convS << 4) + convT;      // config I/V ch in 0x24 command format -- nibble 1 = # of samples; nibble 2 = conv time
  
  ioffT->vout = vout;   // this value is displayed in the ioffsetTable; the real vout is read with statsAllVout_ISL and may be read with 1F 84, 1F 85 etc.

  deltaT = time_x_samples[convT][convS] + 10;   // 10ms over isl conversion time
  
  if (vout < 1470)      // Vdd2, Vddq < 1.5V
    for(int j=0; j<12; j++)     SetVout(vout, i2xy[j]);
  else  {
    for(int j=4; j<8; j++)      SetVout(vout, i2xy[j]);
  }

// Vout only for scale 2: Auxiliary Channel VBUS Voltage -- measures VXS (remote sense V)
  statsAllVout_ISL((u16) 10, deltaT, (u8) 2, &isl_V2);      // only avr needed

  if (scale > 1)  {       // READ Vshunt for Scale 2 - 200mOhm Rshunt
    statsAllVshunt_ISL((u16)3, deltaT, (u8) 2, absI_Max_200, &isl_I21);
    statsAllVshunt_ISL(count, deltaT, (u8) 2, absI_Max_200, &isl_I21);
  }
  else  {       // READ Vshunt for Scale 1 - 80mOhm Rshunt
    statsAllVshunt_ISL((u16)3, deltaT, (u8) 1, absI_Max_80, &isl_I21);
    statsAllVshunt_ISL(count, deltaT, (u8) 1, absI_Max_80, &isl_I21);
  }

  for(int j=0; j<12; j++)     {
    ioffT->vout_avr[j] = isl_V2.avr[j];
    ioffT->offset[j] = (int16_t) isl_I21.avr[j];
    isl->vout[j] = isl_V2.avr[j];
    isl->avr[j] = isl_I21.avr[j];
    isl->pk2pk[j] = isl_I21.pk2pk[j];
    isl->dv[j] = isl_I21.dv[j];
  }
}

// end I Offset ^^^         ###########################################


// I Offset     vvv     #####################################################

//      750     950     1000    1100    1200    1300    1450
//      750     950     1000    1100    1200    1300    1500    1700    1800    1900    2000    2100       only for VX21 .. VX24

//      v0      v1      v2      v3      v4      v5      v6      v7
//      750     850     950     1050    1150    1250    1350    1450

void filloffset(u16 vout, u16 count)
{
//      dt = 0x10 = 16ms        -- use 24 26 -> 4x2048ms = 8ms
  ioffset(      vout,     count, 2, &ioff_v0, &ioffTab_v0);        // count: # of samples for stats; dt: delay between iterations (giving time for a new measurement); scale = 1,2
  ioffset((u16) vout+100, count, 2, &ioff_v1, &ioffTab_v1);        // count: # of samples for stats; dt: delay between iterations (giving time for a new measurement); scale = 1,2
  ioffset((u16) vout+200, count, 2, &ioff_v2, &ioffTab_v2);        // count: # of samples for stats; dt: delay between iterations (giving time for a new measurement); scale = 1,2
  ioffset((u16) vout+300, count, 2, &ioff_v3, &ioffTab_v3);        // count: # of samples for stats; dt: delay between iterations (giving time for a new measurement); scale = 1,2
  ioffset((u16) vout+400, count, 2, &ioff_v4, &ioffTab_v4);        // count: # of samples for stats; dt: delay between iterations (giving time for a new measurement); scale = 1,2
  ioffset((u16) vout+500, count, 2, &ioff_v5, &ioffTab_v5);        // count: # of samples for stats; dt: delay between iterations (giving time for a new measurement); scale = 1,2
  ioffset((u16) vout+600, count, 2, &ioff_v6, &ioffTab_v6);        // count: # of samples for stats; dt: delay between iterations (giving time for a new measurement); scale = 1,2
  ioffset((u16) vout+700, count, 2, &ioff_v7, &ioffTab_v7);        // count: # of samples for stats; dt: delay between iterations (giving time for a new measurement); scale = 1,2
}



// v    ^^^

  u16 value=0;
/*
  void AD5592_Init_8SPI(void)
  {
  	//     General-Purpose Control Register
  	  value =0x1800;                //     data for each channel
  	  DAC_WriteReg(value,0);        //	SPI1	Group C
  	  DAC_WriteReg(value,1);        //	SPI2	Group D
  	  DAC_WriteReg_8SPI(value,2);        //	SPI3	Group B
  	  DAC_WriteReg(value,3);        //	SPI4	Group A

  	  value =0x2807;                //      DAC Pin Configuration	set I/O 0-2 to DAC  = 07
  	  DAC_WriteReg(value,0);        //
  	  DAC_WriteReg(value,1);        //
	  DAC_WriteReg_8SPI(value,2);        //
  	  DAC_WriteReg(value,3);        //

  /* Power-Down/Reference Control Register:
  	  	D15	D14	D10	D9	D8	D7-D0
  	  	0	1011	0	1	0	0000 0000
  	  		Reg. addr.	1: the reference and its buffer are powered up
  					0: the reference and DACs power-down states are determined by D9 and D7 to D0 (default)
  	  					Rsrvd	0: the channel is in normal operating mode (default)
  *
  	value = 0x5A00;
  	DAC_WriteReg(value,0);
  	DAC_WriteReg(value,1);
	DAC_WriteReg_8SPI(value,2);
  	DAC_WriteReg(value,3);

  	/*   write Readback and LDAC Mode Register
  		D15	D14	D10	D6	D5-D2	D1-D0
  		0	0111	0000	1	0101	00
  			Reg. addr.	Enbl Readback	LDAC mode
  				Reserved 	Reg. to be read back = DAC Pin config
  		0	0111	0000	1	1011	00
  	                                        D5-D2=1011: power-down and reference control
  	*
  	  value = 0x386C;	// Readback and LDAC Mode Register
  		DAC_WriteReg(value,0);
  		DAC_WriteReg(value,1);
  		DAC_WriteReg_8SPI(value,2);
  		DAC_WriteReg(value,3);

  	value =0x0000;                //Nop   ?       power-down and reference control
  	DAC_WriteReg(value,0);
  	DAC_WriteReg(value,1);
	DAC_WriteReg_8SPI(value,2);
  	DAC_WriteReg(value,3);

  	  value = 0x3854;//      Readback and LDAC Mode Register
  		DAC_WriteReg(value,0);
  		DAC_WriteReg(value,1);
		DAC_WriteReg_8SPI(value,2);
  		DAC_WriteReg(value,3);
  	  value =0x0000;                //Nop   ?       read DAC Pin config reg
  		DAC_WriteReg(value,0);
  		DAC_WriteReg(value,1);
		DAC_WriteReg_8SPI(value,2);
  		DAC_WriteReg(value,3);
  }
*/

//##############################################################
//
//	AD5592 Setting:
//
//		For this section, use SPI setting:	High, 1Edge
//
//##############################################################
/*	Page 26:
  	  	A write sequence begins by bringing the SYNC line low.
  	  	Data on SDI is clocked into the 16-bit shift register on the falling edge of SCLK.
  	  	After the 16th falling clock edge, the last data bit is clocked in.
  	  	SYNC is brought high, and the programmed function is executed (that is, a change in a DAC input register or a change in a control register).
  	  	SYNC must be brought high for a minimum of 20ns before the next write.

  	  	Reading from a register first requires a write to the readback and LDAC mode register to select the register to read back.
  	  	The contents of the selected register are clocked out on the next 16 SCLKs following a falling edge of SYNC.

  	  	Table 14. Control Register Map		page 26
*/
void AD5592_Init(void)
{
	 uint16_t value;
	 uint16_t rvalue;
//	Table 14. Control Register Map		page 26

// AD5592 Reset pin
		HAL_GPIO_WritePin(GPIOA, 10, GPIO_PIN_RESET);	// RST_DAC2
		HAL_GPIO_WritePin(GPIOD, 13, GPIO_PIN_RESET);	// RST_DAC1
		for(int time=0; time<10; time++) {};
		HAL_GPIO_WritePin(GPIOA, 10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, 13, GPIO_PIN_SET);

//	Software Reset Register
 	  value =0x7DAC;
 	  DAC_WriteReg_16SPI(value,0);        // to U12 DAC1
 	  DAC_WriteReg_16SPI(value,1);        // to U13 DAC2


//	General-Purpose Control Register		Page 28
//			D[15:10]=0 0011 0
//			D[9:6]	see details Page 28
//			D5=		0: ADC gain is 0 V to VREF (default);			1: ADC gain is 0 V to 2 × VREF
//			D4=		0: DAC output range is 0 V to VREF (default);	1: DAC output range is 0 V to 2 × VREF
//			D[3:0]=0000		Reserved

  	  value =0x1800;                //     DAC and ADC control register -- data for each channel

  	  DAC_WriteReg_16SPI(value,0);
  	  DAC_WriteReg_16SPI(value,1);

//	DAC Pin Configuration	Page 29
//			D[15:8]=0 0101 000
//			D[7:0]	DAC7 to DAC0	=	1: I/Ox is a DAC output		0: I/Ox function is determined by the pin configuration registers (default)

  	  value =0x280F;                // DAC Pin Configuration	set I/O 0:3 to DAC  = 0F
//															(for U13, IO4 is DAC_V3V3 to U5 - not in use)
  	  DAC_WriteReg_16SPI(value,0);        // to U12 DAC1
  	  DAC_WriteReg_16SPI(value,1);        // to U13 DAC2

  	  value =0x20FF;                // ADC Pin Configuration	set all I/O ADC (= FF)

  	  DAC_WriteReg_16SPI(value,0);
  	  DAC_WriteReg_16SPI(value,1);


// Power-Down/Reference Control Register.	PD_REF_CTRL
//	Powers down DACs and enables/disables the reference
//	value =0x5A00;		// 00 is OK too. The channel is powered down if it is configured as a DAC
  	value = 0x5AF0;		// F0:	DAC7..DAC4 powered down; DAC3..DAC0 in normal operating mode
	DAC_WriteReg_16SPI(value,0);
	DAC_WriteReg_16SPI(value,1);



/* Power-Down/Reference Control Register:
  	  	D15	D14	D10	D9	D8	D7-D0
  	  	0	1011	0	1	0	0000 0000
  	  		Reg. addr.	1: the reference and its buffer are powered up
  					0: the reference and DACs power-down states are determined by D9 and D7 to D0 (default)
  	  					Rsrvd	0: the channel is in normal operating mode (default)
*/
  	value = 0x5AF0;		// F0:	DAC7..DAC4 powered down; DAC3..DAC0 in normal operating mode (00 is OK too: The channel is powered down if it is configured as a DAC)
  	DAC_WriteReg_16SPI(value,0);
  	DAC_WriteReg_16SPI(value,1);


// Testing:	vvv
//  	value = 0x1AD3;
//	hstatus = DAC_SPI_TransmitReceive(value, &rvalue, 0);
//	value = 0x1AD3;
//	  	DAC_WriteReg_16SPI(value,0);
//	  	hstatus = DAC_ReadReg_16SPI(&rvalue, 0);
// End Testing:	^^^


// Testing:

/*   write Readback and LDAC Mode Register	Page 40
		D[15:7]=0011 1000 0
		D6=		0: no readback is initiated;	1: Bit D5 to Bit D2 select which register is read back
			Bit D6 automatically clears when the read is complete.
		D[5:2]	register to be read back	See page 40 for details
		D[1:0]	LDAC Mode	See page 40

		D[5:2]=0101		Reg. to be read back = DAC Pin config
		D[5:2]=1011		Reg. to be read back = power-down and reference control
*/
  	rvalue = 0x55AA;
  	value = 0x386C;	// D6=1;	D[5:2]=1011;	D[1:0]=00	read back power-down and reference control
//	hstatus = DAC_SPI_TransmitReceive(value, &rvalue, 0);
//221123	  	DAC_WriteReg_16SPI(value,0);
//221123	  	hstatus = DAC_ReadReg_16SPI(&rvalue, 0);

//  	value =0x0000;                //Nop   ?
//  	DAC_WriteReg_16SPI(value,0);
//  	DAC_WriteReg_16SPI(value,1);

  	rvalue = 0xAA55;
	value = 0x3854;	// D6=1;	D[5:2]=0101;	D[1:0]=00	read back DAC Pin config
//	hstatus = DAC_SPI_TransmitReceive(value, &rvalue, 0);
//221123	  	DAC_WriteReg_16SPI(value,0);
//221123	  	hstatus = DAC_ReadReg_16SPI(&rvalue, 0);


// Write DAC Data
  	value = 0x9800;		// for DAC1		-- D[14:12]: DAC channel; Data = 0x800
  	value = 0x5AF0;		// test F0:	DAC7..DAC4 powered down; DAC3..DAC0 in normal operating mode (00 is OK too: The channel is powered down if it is configured as a DAC)
  	DAC_WriteReg_16SPI(value,0);

//		DAC READBACK
//	To read back a DAC input register, it is first necessary to enable
//	the readback function and select which DAC register is required.
//	This is achieved by writing to the DAC read back register (see Table
//	15). Set the Bits[4:3] to 11 to enable the readback function. Bits[2:0]
//	select which DAC data is required. The DAC data is clocked out of
//	the AD5592R/AD5592R-1 on the subsequent SPI operation. Figure
//	44 shows an example of setting I/O3, configured as a DAC, to midscale.
//	The input data is then read back. Bits[14:12] contain the
//	address of the DAC register being read back, and Bit 15 is 1 (see Table 32).

// 	Table 15. Bit Descriptions for DAC_RD		Page 36
//  	value = 0x0818;		// ??	for DAC0		-- D[2:0]: DAC channel
	value = 0x0809;		// for DAC1		-- D[2:0]: DAC channel
//	hstatus = DAC_SPI_TransmitReceive(value, &rvalue, 0);
	DAC_WriteReg_16SPI(value,0);
	hstatus = DAC_ReadReg_16SPI(&rvalue, 0);

	  value =0x0000;                //Nop   ?
//  		DAC_WriteReg_16SPI(value,0);
//  		DAC_WriteReg_16SPI(value,1);

}


//        DAC, AD5592 init    old -- from DA
void DAC_Init(void)
{
//  u16 value=0;
//  u16 id;       //      for testing

//vvv  Reset_DAC();
  
//      DAC00    4-7 for ADC
//  value =0x20f0;              //      ADC Config
//  DAC_WriteReg(value,id);

  
//      DAC00   0-3 for DAC

//v     General-Purpose Control Register
//  value =0x1840;              //v     all channels same data
  value =0x1800;                //v     data for each channel
  DAC_WriteReg(value,0);        //v     u5

  value =0x280f;                //      DAC Pin Configuration
  DAC_WriteReg(value,0);        //      u5:     set 0-3 to dac  = 0F

/* Power-Down/Reference Control Register:
	D15	D14	D10	D9	D8	D7-D0
	0	1011	0	1	0	0000 0000
		Reg. addr.	1: the reference and its buffer are powered up
			0: the reference and DACs power-down states are determined by D9 and D7 to D0 (default)
					Rsrvd	0: the channel is in normal operating mode (default)
*/
  value = 0x5A00;
  DAC_WriteReg(value,0);        //      u5

  
/*   write Readback and LDAC Mode Register
	D15	D14	D10	D6	D5-D2	D1-D0
	0	0111	0000	1	0101	00
		Reg. addr.	Enbl Readback	LDAC mode
			Reserved 	Reg. to be read back = DAC Pin config
	0	0111	0000	1	1011	00
                                        D5-D2=1011: power-down and reference control
*/
  value = 0x386C;
  DAC_WriteReg(value,0);        //      u5      Readback and LDAC Mode Register
  value =0x0000;                //Nop   ?
  value = DAC_ReadReg(0);        //      ?       power-down and reference control
  
  value = 0x3854;
  DAC_WriteReg(value,0);        //      u5      Readback and LDAC Mode Register
  value =0x0000;                //Nop   ?
  value = DAC_ReadReg(0);        //      ?       power-down and reference control


//      DAC12   2-7 for DAC     u37

//     General-Purpose Control Register
//  value =0x1840;              //v     all channels same data
  value =0x1800;                //v     data for each channel
  DAC_WriteReg(value,1);        //v     u5

//      DAC Pin Configuration
//  value =0x28fc;        //      u37:     set 7-2 to dac  = FC; DAC0/1 NC
  value =0x28ff;        //      u37:     set 7-0 to dac  = FF
  DAC_WriteReg(value,1);

// Power-Down/Reference Control Register:
  value = 0x5A00;
  DAC_WriteReg(value,1);        //      u37

  
//   write Readback and LDAC Mode Register
  value = 0x386C;
  DAC_WriteReg(value,1);        //      u37      Readback and LDAC Mode Register
  value =0x0000;                //Nop   ?
  value = DAC_ReadReg(1);        //      ?       power-down and reference control
  
  value = 0x3854;
  DAC_WriteReg(value,1);        //      u37      Readback and LDAC Mode Register
  value =0x0000;                //Nop   ?
  value = DAC_ReadReg(1);        //      ?       read DAC Pin config reg

  
//      DAC23   2-7 for DAC     u6

//     General-Purpose Control Register
//  value =0x1840;              //v     all channels same data
  value =0x1800;                //v     data for each channel
  DAC_WriteReg(value,2);        //v     u5

//      DAC Pin Configuration
//  value =0x28fc;        //      u6:     set 7-2 to dac  = FC; DAC0/1 NC
  value =0x28ff;        //      u6:     set 7-0 to dac  = FF
  DAC_WriteReg(value,2);

// Power-Down/Reference Control Register:
  value = 0x5A00;
  DAC_WriteReg(value,2);        //      u6

  
//   write Readback and LDAC Mode Register
  value = 0x386C;
  DAC_WriteReg(value,2);        //      u6      Readback and LDAC Mode Register
  value =0x0000;                //Nop   ?
  value = DAC_ReadReg(2);        //      ?       power-down and reference control
  
  value = 0x3854;
  DAC_WriteReg(value,2);        //      u6      Readback and LDAC Mode Register
  value =0x0000;                //Nop   ?
  value = DAC_ReadReg(2);        //      ?       read DAC Pin config reg
}



/* 0x22   Jmp to boot loader */
void ToBootLoader(void)
{
   HAL_FLASH_Unlock();
   HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,0x0000C000, 0x00000000);
   HAL_FLASH_Lock();
   NVIC_SystemReset();   
}

/*	0x21    Reset system
void Reset_System(void)
{
   NVIC_SystemReset();
}
*/

u32 Xo2_write_read_test(u8 regAddr) 
{
  u8  cmd[10]= {0x02,0xA1,0xa5,0x50,0x51,0x52,0x53,0x54,0x55,0x56};
//              write regAddr=A1
  u8 data[10]={0xAA};
  
  cmd[0] = 0x02;     // write register regAddr 
  cmd[1] = regAddr;
//  status = I2C_LB_RW(cmd,10,data,0,I2C_LBMachXo2);
//  status = I2C_ISL(&hi2c2,cmd,10,data,0,I2C_LBMachXo2);

  cmd[0] = 0x0B;     // read register regAddr  
//  status = I2C_ISL(&hi2c2,cmd,2,data,8,I2C_LBMachXo2);

  if (status == OK)
  {
    status = (data[0]<<24) | (data[1]<<16) | (data[2]<<8) | data[3];
  }

  return status;
}

u32 Xo2_write(u8 regAddr) 
{
  u8  cmd[10]= {0x02,0xA1,0xa5,0x50,0x51,0x52,0x53,0x54,0x55,0x11};
  u8 data[10]={0xAA};
  
  cmd[0] = 0x02;     // write register regAddr 
  cmd[1] = regAddr;  
//  status = I2C_LB_RW(cmd,10,data,0,I2C_LBMachXo2);
//  status = I2C_ISL(&hi2c2,cmd,10,data,0,I2C_LBMachXo2);

  if (status == OK)
  {
    status = (data[0]<<24) | (data[1]<<16) | (data[2]<<8) | data[3];
  }
  return status;
}

u32 Xo2_write_onebyte(u8 regAddr, u8 value) 
{
  //u8 cmd[10]= {0xe0,0x00,0x00,0x00};
//  u8  cmd[10]= {0x0b,0x0f,0xa5,0x50,0x50,0x50,0x50,0x50,0x50,0x00};
  u8  cmd[10]= {0x02,0xF0,0xa5,0x50,0x50,0x50,0x50,0x50,0x50,0x00};
//              write regAddr=F0
  u8 data[10]={0xaa};
//  u32 status;
  
  cmd[0] = 0x02;     // 0x02 write memory - 8 bytes write data 
  cmd[1] = regAddr;     // 0xF0 for LED register  
  cmd[2] = value;  
  status = I2C_LB_RW(cmd,10,data,0,I2C_LBMachXo2);

  if (status == OK)
  {
    status = (data[0]<<24) | (data[1]<<16) | (data[2]<<8) | data[3];
  }
  return status;
}


u32 Xo2prim0_read(u8 value)     // OK 
{
  u8  cmd[10]= {0xE0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  u8 data[10]={0xaa};
//  u32 status;

  cmd[0] = value;     // 0xE0 for device ID, 0x19 for TraceID  
  status = I2C_LB_RW(cmd,1,data,8,I2C00_LBMachXo2);     //      #define I2C00_LBMachXo2 0x80

  if (status == OK)
  {
    status = (data[0]<<24) | (data[1]<<16) | (data[2]<<8) | data[3];
  }
  return status;
}


u32 Xo2prim_read(u8 value) 
{
  u8  cmd[10]= {0xE0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  u8 data[10]={0xaa};

  cmd[0] = value;     // 0xE0 for device ID, 0x19 for TraceID  
  status = I2C_LB_RW(cmd,1,data,8,I2C0_LBMachXo2);      //      #define I2C0_LBMachXo2 0x82 

  if (status == OK)
  {
    status = (data[0]<<24) | (data[1]<<16) | (data[2]<<8) | data[3];
  }
  return status;
}


  //      u32 I2C_ISL( I2C_HandleTypeDef* I2Cx,u8 * pIn, u16 NumByteToWrite,u8 * pOut,u16 NumByteToRead,u8 I2C_ADDRESS)
u32 Xo2_write_str(u8 regAddr, u8 * str) 
{
  u8  cmd[10];
  u8  read_str[10];
  int i;

  cmd[0] = 0x02;        // 0x02 - write memory - 8 bytes  
  cmd[1] = regAddr;  
//  cmd[9 downto 2] = str[7 downto 0];  
//  cmd[9:2] = str[7:0];
  for (i = 0; i < 8; ++i)  
    cmd[i + 2] = str[i];
  
//  status = I2C_ISL(&hi2c2, cmd, 10, &read_str[0], 0, I2C_LBMachXo2);     //      #define I2C_LBMachXo2 0x84

  return status;
}


u32 Xo2_read_TraceID(u8 * read_str) 
{
/*
TN1207_UsingTraceID, page 4:
1. Send start condition.
2. Send default slave address (8�h80) and write command.
3. Send the 8-bit command 8�h19.
4. Send the 24-bit operand 24�h000000 in three single-byte transfers.
5. Send repeated start.
6. Send the slave address and read command.
7. Read the first byte of the TraceID and send ack.
8. Read the second to seventh bytes of the TraceID and send ack for each byte read.
9. Read the last TraceID byte and send nack.
10.Send the stop command.
*/
// returns      A5 44 33 72 77 50 A8 2A         for XO2-640 TQFP-100    module 0029
  u8  cmd[5];

  cmd[0] = 0x19;  
  cmd[1] = 0x00;  
  cmd[2] = 0x00;  
  cmd[3] = 0x00;  
//  status = I2C_LB_RW(cmd,5,data,8,I2C00_LBMachXo2);     //      #define I2C00_LBMachXo2 0x80
//  status = I2C_ISL(&hi2c2, cmd, 4, &read_str[0], 8, I2C00_LBMachXo2);     //      #define I2C00_LBMachXo2 0x80

  return status;
}

u32 Xo2_read_DeviceCode(u8 * read_str) 
{
  //      TN1204_MachXO2ProgrammingandConfigurationUsageGuide.pdf, page 51
  //  Read Device ID      [IDCODE_PUB]
  //    command: 0xE0       read data: 4 bytes
  //    Read Device ID [IDCODE_PUB]:	80 e0 00 00 00
          //    Restult: 01 2B 80 43    for XO2-256 QFN-32
          //    Restult: 01 2B 90 43    for XO2-640 TQFP-100

  u8  cmd[5];

  cmd[0] = 0xE0;  
  cmd[1] = 0x00;  
  cmd[2] = 0x00;  
  cmd[3] = 0x00;  
//  status = I2C_ISL(&hi2c2, cmd, 4, &read_str[0], 4, I2C00_LBMachXo2);     //      #define I2C00_LBMachXo2 0x80

  status = (read_str[0]<<24) | (read_str[1]<<16) | (read_str[2]<<8) | read_str[3];

  return status;
}


u32 readXo2ConfigID(u8 * read_str)      // 0x80: FPGA config. ID  //read: rev, year, month, day, etc.
{
  status = Xo2_read_str(0x80, read_str);       // 0x80: FPGA config. ID  //read: rev, year, month, day, etc.
  return status;
}


/*  XO2 test  */
void XO2_Test(void)
{
  u8 read_str[8];
  u8  str[10]= {0x02,0xA1,0xa5,0x50,0x51,0x52,0x53,0x54,0x55,0x11};
    
// reset I2C XO2 -- MCU commnd, vector(0,0,0,0)
  // vector(0,0,0,0) - PE11, PE15, PC4, PD13
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_Delay(200);
  // MCU commnd: PE8 0-1
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET); 
  HAL_Delay(200);

    BSP_LED_Off(Led1);
    BSP_LED_Off(Led2);
    HAL_Delay(1000);

    readXo2ConfigID(read_str);     // 0x80: FPGA config. ID;  read: rev, year, month, day, etc.

    status = Xo2_write_str(0xA1, str);  // 0xA1: Alert Mask 
    status = Xo2_read_str(0xA1, &read_str[0]);  // 0xA1: Alert Mask

    status = Xo2_read_TraceID(read_str);       // I2C adddr = 0x80; 0x19: FPGA TraceID

    status = Xo2_read_DeviceCode(read_str);       // I2C adddr = 0x80; 0xE0: FPGA ID 

    Xo2prim0_read(0xE0);       // 0xE0: FPGA ID
    Xo2prim_read(0xE0);       // 0xE0: FPGA ID
    status = Xo2_read_str(0xE0, &read_str[0]);       // 0xE0: FPGA ID
    BSP_LED_Toggle(Led1);
    HAL_Delay(100);

    status = Xo2_read_str(0x80, &read_str[0]);      // 0x80: FPGA config. ID;  read: rev, year, month, day, etc. 

    Xo2prim0_read(0x19);       // 0x19: FPGA TraceID
    Xo2prim_read(0x19);       // 0x19: FPGA TraceID
    status = Xo2_read_str(0x19, &read_str[0]);       // 0x19: FPGA TraceID
    BSP_LED_Toggle(Led2);
    HAL_Delay(100);

    status = Xo2_read_str(0xA1, &read_str[0]);  // 0xA1: Alert Mask

    Xo2_write_read_test(0xA1); 

}


void set_allVxy(u16 vout)
{
 u16 vout_4644;
 
//      void SetVout_4644(u16 vout, u8 id)	// id = 0, 1, 2, 3	for Vout1/2/3/4
//      void SetVout_3026(u16 vout, u8 id)	// id = 11,12,13,14, 21,22,23,24, 31,32,33,34

  vout_4644 = vout + 300;
  
  SetVout_4644(vout_4644,0); // Vout1 to vx11, vx12
  SetVout_4644(vout_4644,1); // Vout2 to vx21, vx22 ... vx24
  SetVout_4644(vout_4644,2); // Vout3 to vx13, vx14, vx31, vx32
  SetVout_4644(vout_4644,3); // Vout4 to vx33, vx34

  HAL_Delay(200);

  for(int i = 0; i<12; i++)    {
    SetVout_3026(vout, i2xy[i]);
  }
}




uint8_t conv_data[4];
uint8_t convTime, convAvr, data2bit7;

void WriteConvCh(u8 cnt)
{
// uint8_t convTime, convAvr, data2bit7;

        convTime = cnt & 0x07;
        convAvr = cnt & 0xF0;
        
        data2bit7 = (cnt & 0x01) << 7;
        
        conv_data[2] = data2bit7 + convTime + (convAvr >> 1);
        conv_data[1] = (convAvr >> 2) + (convTime >> 1);

        conv_data[0] = 0xD5; 					// Config V Channels -- R/W
	for (int i = 0; i < 12; i++) {
//          status = I2C_ISL(&hi2c3, conv_data, 3, NULL, 0, i2c_Vxy[i]);  // write config V channels reg
	}

	conv_data[0] = 0xD4;  					// Config I Channels  -- 2 bytes R/W    default = 0387
	for (int i = 0; i < 12; i++) {
 //         status = I2C_ISL(&hi2c3, conv_data, 3, NULL, 0, i2c_Vxy[i]);  // write config I channels reg
	}
}



void Test0()
{
   u16 value=0;
   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET); // no mcu command to FPGA
   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); // LTM4644 enabled 
   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // to disable The FPGA SPI I/F
   HAL_Delay(200); // 0.2 seconds delay
     

// reset I2C Xo2 -- MCU commnd, vector(0,0,0,0)
  // vector(0,0,0,0) - PE11, PE15, PC4, PD13
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
  // MCU commnd: PE8 0-1
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET); 
  HAL_Delay(200);

  set_LPDDR4();
  
   HAL_Delay(2000);
 
}       // end Test0


#define VersionH      0x02
#define VersionL      0x01
#define Version_vH    0x12
//#define Version_vL    0xAA    // normal while(1)   -- before reaching the while(1), sets power to pins, ignoring Socket board powering
//#define Version_vL    0xCC    // normal while(1)
//#define Version_vL    0xBF    // has its own while(1)   -- pin powers depending of Socket board powering
//#define Version_vL    0xBB    // has its own while(1)   -- tests OC, alert
//#define Version_vL    0xDD    // has its own  while(1)   -- runs stats inside the while loop; pin powers to std depending of Socket board powering
#define Version_vL    0x23        // normal while(1)

uint32_t value32;
uint16_t value16;


u32 version_FW(u8 * read_str)
{
  read_str[0] = VersionL;
  read_str[1] = VersionH;
  read_str[2] = Version_vL;
  read_str[3] = Version_vH;
//  status = Xo2_read_str(0x80, read_str);       // 0x80: FPGA config. ID  //read: rev, year, month, day, etc.
//  FWversion = (VersionH * 256 + VersionL) * 256 + Version_vL;
  FWversion = (((Version_vH * 256 + Version_vL) * 256) + VersionH) * 256 + VersionL;
  return (FWversion);
}


u8 ex_txBuff[TXBUFFER_SIZE];

/*
struct strct12          isl_V1, isl_V2;
struct strct12x32       isl_I1, isl_I2, isl_I21;
struct offstrct12x32    ioff_v1, ioff_v2, ioff_v3, ioff_v4, ioff_v5, ioff_v6;
struct ioffsetTable     ioffTab_v1, ioffTab_v2, ioffTab_v3, ioffTab_v4, ioffTab_v5, ioffTab_v6;
*/

void displayIavr(struct strct12x32 * isl, int j)
{
  for (int i = 0; i < 6; ++i)   {
    ex_txBuff[4*i] = 0;                              // MSB -- I < 2**24 but may be > 2**16
    ex_txBuff[4*i + 1] = (u8) (isl->avr[i+j] >> 16);
    ex_txBuff[4*i + 2] = (u8) (isl->avr[i+j] >> 8);
    ex_txBuff[4*i + 3] = (u8) isl->avr[i+j];        // LSB
  }
}


void displayIoffTab(struct ioffsetTable * ioffTab)      // 4+24=28 bytes = 0x1C
{
  ex_txBuff[0] = (u8) (ioffTab->convI >> 8);
  ex_txBuff[1] = (u8) ioffTab->convI;
  ex_txBuff[2] = (u8) (ioffTab->vout >> 8);
  ex_txBuff[3] = (u8) ioffTab->vout;
/*          for (int i = 0; i < 12; ++i)   {  
            ex_txBuff[i+4] = ioffTab->offset[i];
*/
  for (int i = 0; i < 12; ++i)   {  
    ex_txBuff[2*i+4] = (u8) (ioffTab->offset[i] >> 8);    // MSB
    ex_txBuff[2*i+5] = (u8) ioffTab->offset[i];
  }
}

void displayV4offTab(struct ioffsetTable * ioffTab)
{
  for (int i = 0; i < 12; ++i)   {  
    ex_txBuff[2*i] = (u8) (ioffTab->vout_avr[i] >> 8);    // MSB
    ex_txBuff[2*i+1] = (u8) ioffTab->vout_avr[i];
  }
}


void displayIoff_avr(struct offstrct12x32 * ioff_v)      // 12x3=36 bytes
{
          for (int i = 0; i < 12; ++i)   {
//            ex_txBuff[4*i] = 0;                              // MSB -- I < 2**24 but may be > 2**16
            ex_txBuff[3*i + 0] = (u8) (ioff_v->avr[i] >> 16);
            ex_txBuff[3*i + 1] = (u8) (ioff_v->avr[i] >> 8);
            ex_txBuff[3*i + 2] = (u8) ioff_v->avr[i];        // LSB     struct offstrct12x32
          }
}


void displayIoff_pk2pk(struct offstrct12x32 * ioff_v)      // 12x3=36 bytes
{
          for (int i = 0; i < 12; ++i)   {
//            ex_txBuff[4*i] = 0;                              // MSB -- I < 2**24 but may be > 2**16
            ex_txBuff[3*i + 0] = (u8) (ioff_v->pk2pk[i] >> 16);
            ex_txBuff[3*i + 1] = (u8) (ioff_v->pk2pk[i] >> 8);
            ex_txBuff[3*i + 2] = (u8) ioff_v->pk2pk[i];        // LSB
          }
}


void displayI(u32 inst[12], int j)
{
          for (int i = 0; i < 6; ++i)   {
            ex_txBuff[4*i]     = (u8) (inst[i+j] >> 24);
            ex_txBuff[4*i + 1] = (u8) (inst[i+j] >> 16);
            ex_txBuff[4*i + 2] = (u8) (inst[i+j] >> 8);
            ex_txBuff[4*i + 3] = (u8) inst[i+j];        // LSB
          }
}


//float finstI;
u16 finst;




 u8 cmd_flag;
 uint16_t value;



