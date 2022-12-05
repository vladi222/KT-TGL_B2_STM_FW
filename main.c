/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c	221128	1:08 PM
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdlib.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "vsv_functions.h"
#include "functions.h"
#include "boardspecs.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi4;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI4_Init(void);
/* USER CODE BEGIN PFP */

void AD5592_Init(void);
void AD5592_Init_8SPI(void);
void SetVDD_4671(u16 vout, u8 id);		// id for DAC dev.	0 to 3
void SetVPP_4671(u16 vout, u8 id);		// id for DAC dev.	0 to 3
void SetVDDQ_4671(u16 vout, u8 id);		// id for DAC dev.	0 to 3	Group C, D, B, A
HAL_StatusTypeDef DAC_WriteReg(u16 value, int id);
uint16_t DAC_ReadReg(int id);
HAL_StatusTypeDef DAC_WriteReg_8SPI(u16 value, int id);
HAL_StatusTypeDef DAC_ReadReg_8SPI(u16 * value, int id);
HAL_StatusTypeDef ADC_7124_WriteReg(u16 value, int id);
HAL_StatusTypeDef ADC_7124_WriteReg_3(uint8_t cmd, u32 value, int id);
uint8_t ADC_7124_ReadReg(uint8_t cmd, int id);
//HAL_StatusTypeDef ADC_7124_ReadReg(u8 cmd, u16 * value, int id)
HAL_StatusTypeDef ADC_7124_WriteReg_2(uint8_t cmd, u16 value, int id);	// for 8-bit SPI
uint32_t ADC_7124_ReadReg_2(uint8_t cmd, int id);
uint32_t ADC_7124_ReadReg_3(uint8_t cmd, int id);
uint16_t vsv_serial_det(int fpga, uint8_t rw, uint16_t A15_14, uint16_t CH_NO, uint16_t A5_0, uint16_t d, uint8_t target);
HAL_StatusTypeDef ADC_7124_WriteReg_2(uint8_t cmd, u16 value, int id);
void vsv_reset(void);

void FPGA1_Write(uint32_t addr, uint8_t data);
uint8_t FPGA1_Read(uint32_t addr);

void read_fpga_id(uint8_t * FPGA_ID);
void fpga_reset(void);

void vsv_serial(int id, uint8_t rw, uint16_t addr, uint16_t d, uint8_t target);
void vsv_init(void);

int tgl_b2_test(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


//	KT-TGL B2	###############################

	  uint8_t dac_io;

		int a, b, c;
		uint16_t vsv_addr;
		uint16_t value;
		uint16_t DACcode, DACcode0;

//		uint16_t DACcode_VIN, DACcode_VDD1, DACcode_VDDQ, DACcode_VDD2, VSVcode_VDD1, VSVcode_VDDQ;

		uint32_t addr;

		uint8_t FPGA_ID[8];
		uint8_t F1_RST, F1_EN, F1_OT, SVSV_STAT0, SVSV_STAT;
		uint8_t F1_VSV_ID[8];	//, F2_VSV_ID[32];
		uint8_t F1_AAB[8], F1_ACD[8], F1_BAB[8], F1_BCD[8];
		uint8_t F1_AAB_ID[8];	//, F1_ACD_ID[8], F1_BAB_ID[8], F1_BCD_ID[8];

		uint16_t DAC_VDD1, DAC_VDDQ;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */


  // #########################################################

  	a = 0;

  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // to disable the FPGA SPI I/F -- SPI5_AB_CS#
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // to disable the FPGA SPI I/F -- SPI5_CD_CS#

  	fpga_reset();

  	addr = 0x00F0;	// LEDs FPGA
  	FPGA1_Write(addr, 0x09);

//##############################################################
//	Read FPGA ID:
//##############################################################
  	fpga_reset();				// PA0: set - reset - set
	for (int i = 0; i<8; i=i+1)		FPGA_ID[i] = (uint8_t) i;
  	read_fpga_id(&FPGA_ID);		//	FPGA1_Read(0x80 + i);


//##############################################################
//	AD5592 Setting:
//##############################################################
	AD5592_Init();


//##############################################################
//	Read VSV Die ID:		page 65		Addr = 0xC07F
//			Addr[15:0]:	0xC07F		1 1 0 0		0 0 0 0		0 1 1 1		1 1 1 1
//
//		read-back:	Page 59		D[15:4]: Product ID (0xF10);	D[3:0]: Die Revision (0x2)
/*	Expected:
 * 		svsv_r(J)[23:0]:	xxxx xx	11	1100 01 00		00	00 10 xx
				D[15:4]:			11	1100 01 00		00				0xF10
	  			D[3:0]:										00 10		0x2
*/
//	Typical return:		0xb		0xc4		0b xxxx xx11	--	0xB - 0xC4 - 0x3 (or 0x7 or 0xB, etc.)
//					0000 1011	1100 0100	xxxx xx11
//
//	F1_VSV_ID =		{addr(0x50), addr(0x51), addr(0x52)};
//	Typical return:		0xb			0xc4	0b xxxx xx11
//					0000 1011	1100 0100	xxxx xx11
//
//	{addr(0x52), 	addr(0x51), 	addr(0x50):
//	xxxx xx 11		11 00 01 00		00 00 10	11
//
//##############################################################

		  	FPGA1_Write(ADDR_VSV_RST, 0x0);				// Reset (vsv_rst) inactive
	  		FPGA1_Write(ADDR_VSV_EN, 1);				// EN pin = 1, if S/W selected, FORCE_# are enabled	(0: all 8 channels FORCE_# = HiZ)
		  	F1_RST = FPGA1_Read(ADDR_VSV_RST);			// 0xFF: all 8x VSVs reset active
		  	F1_EN = FPGA1_Read(ADDR_VSV_EN);			// 0:	 all 8x VSVs 8 channels	FORCE_# = HiZ
		  	SVSV_STAT0 = FPGA1_Read(ADDR_SVSV_STAT);	// 0:	 VSV serial link inactive

		  	vsv_serial(1, 0, 0xC07F, 0x00, 0xFF);		// to FPGA1 -- read VSV Die ID
		  	SVSV_STAT = FPGA1_Read(ADDR_SVSV_STAT);		// 0:	 VSV serial link inactive?
		  	for (int i = 0; i<8; i=i+1)		F1_AAB_ID[i] = FPGA1_Read(0x20 + i);	// 00 7f c0 00 00	-- VSV Serial Command - addr + data
			for (int i = 0; i<8; i=i+1)		F1_VSV_ID[i] = FPGA1_Read(0x50 + i);	// 0b c4 07		VSV Die ID -- Read from VSV


// #########################################################
//		VSV Setting:
// #########################################################
//
//	Measurement Unit Source	Selection:	A15_14=0b10, A[5:0]=5	(page 62)
		vsv_addr = vsv_serial_det(1, 1, 0b10, 0xFF, 5, 0x1800, 0xFF);	// All Chs; All VSV; IR = IR5 (512mA Range)	-- D12=WE=1, D11=IR#<5>=1

		vsv_addr = vsv_serial_det(1, 1, 0b10, 0xFF, 0, 0x1800, 0xFF);	// All Chs; All VSV; FVR# = 0x00 (8V Range)	-- D10=WE=1, D9=0
		vsv_addr = vsv_serial_det(1, 1, 0b10, 0xFF, 4, 0x500, 0xFF);	// All chs, all VSV -- FORCE_# to SENSE_#
		vsv_addr = vsv_serial_det(1, 1, 0b10, 0xFF, 1, 0xD, 0xFF);		// All Chs Active -- D3=WE=1, D2=Sel-DPS-En#=1, D1=Sel-RT-En=0#, D0=CPU-En#=DSP-En#=1

		vsv_addr = vsv_serial_det(1, 1, 0b10, 0xFF, 1, 0xC, 0xFF);	// HiZ: All Chs, All VSV	// D3=WE=1, D2=Sel-DPS-En#=1, D1=Sel-RT-En#=0, D0=CPU-En#=DSP-En#=0

// #################################################################################################################
	tgl_b2_test();	// TGL B2 Board Power Testing Procedure	--	comment this line if the testing is not needed
// #################################################################################################################


	vsv_addr = vsv_serial_det(1, 1, 0b10, 0xFF, 1, 0xC, 0xFF);	// HiZ: All Chs, All VSV	// D3=WE=1, D2=Sel-DPS-En#=1, D1=Sel-RT-En#=0, D0=CPU-En#=DSP-En#=0


//########################################################
//	Setting the Board to the nominal power voltages:
// #######################################################

/*		now in boardspecs.h
//	For the board 0002:
// ######################################
	DACcode_VIN = 0x800;
	DACcode_VDD1 = 0x700;	// for VIN_DD1 = 2.3V
	DACcode_VDDQ = 0xD00;	// for VIN_DDQ = 2V
	DACcode_VDD2 = 0x400;	// for VDD2_DUT1/2/3/4 = 1.1V
	VSVcode_VDD1 = 0x8C00;	// for VDD1 = 1.8V
	VSVcode_VDDQ = 0x5C00;	// for VDDQ = 0.6V
// ######################################
*/

//	VIN12, VIN34	for	VDD2_DUT#	1.6V	1.2V … 2V		U12/13 IO2:	DAC_IN12/23 = 0x800	for	VIN12/34 = 1.6V
//	VSV		for	VDD1		2.3V	1.9V … 2.7V	U12 IO3:	U12 IO3:	DAC_DD1 = ~ 0x700	for	VIN_DD1 = 2.3V
//	VSV		for	VDDQ		2V 								U13 IO3:	DAC_DDQ = ~ 0xD00	for	VIN_DDQ = 2V

	value = 0x8000 | (2*0x1000) | DACcode_VIN;	// U26, U29
	DAC_WriteReg_16SPI(value, 0);				// U26		VIN12		(12V)	TP: L6 Bottom; C292 Top
	DAC_WriteReg_16SPI(value, 1);				// U29		VIN34		(12V)	TP: L7 Bottom; C321 Top
	value = 0x8000 | (3*0x1000) | DACcode_VDD1;	// U22
	DAC_WriteReg_16SPI(value, 0);				// U22		VIN_DD1		(12V)	TP: D2/3/4/5 Bottom
	value = 0x8000 | (3*0x1000)	| DACcode_VDDQ;	// U23
	DAC_WriteReg_16SPI(value, 1);				// U23		VIN_DDQ		(12V)	TP: D6/7/8/9 Bottom


//	VDD2_DUT1/2/3/4 = 1.1V	U12/U13 IO0, IO1	DAC_DD2_1/2/3/4

	value = 0x8000 | (0*0x1000) | DACcode_VDD2;	// U27, U30
	DAC_WriteReg_16SPI(value, 0);				// U27		VDD2_DUT1	(VIN12)	TP: C316 Bottom; R181.2 Top
	DAC_WriteReg_16SPI(value, 1);				// U30		VDD2_DUT3	(VIN34)	TP: C346 Bottom; R200.2 Top
	value = 0x8000 | (1*0x1000) | DACcode_VDD2;	// U28, U31
	DAC_WriteReg_16SPI(value, 0);				// U28		VDD2_DUT2	(VIN12)	TP: C317, R183 Bottom
	DAC_WriteReg_16SPI(value, 1);				// U31		VDD2_DUT4	(VIN34)	TP: C335 Bottom


	vsv_addr = vsv_serial_det(1, 1, 0b10, 0xFF, 5, 0x1800, 0xFF);	// All Chs; All VSV; IR = IR5 (512mA Range)	-- D12=WE=1, D11=IR#<5>=1

	vsv_addr = vsv_serial_det(1, 1, 0b10, 0xFF, 0, 0x1800, 0xFF);	// All Chs; All VSV; FVR# = 0x00 (8V Range)	-- D10=WE=1, D9=0
	vsv_addr = vsv_serial_det(1, 1, 0b10, 0xFF, 4, 0x500, 0xFF);	// All chs, all VSV -- FORCE_# to SENSE_#
	vsv_addr = vsv_serial_det(1, 1, 0b10, 0xFF, 1, 0xD, 0xFF);		// All Chs Active -- D3=WE=1, D2=Sel-DPS-En#=1, D1=Sel-RT-En=0#, D0=CPU-En#=DSP-En#=1

	vsv_addr = vsv_serial_det(1, 1, 0b00, 0x01, 0, VSVcode_VDD1, 0x10);	// VDD1_DUT1, Ch0:	ForceA#<15:0> = Value
	vsv_addr = vsv_serial_det(1, 1, 0b00, 0x02, 0, VSVcode_VDD1, 0x10);	// VDD1_DUT2, Ch1:	ForceA#<15:0> = Value
	vsv_addr = vsv_serial_det(1, 1, 0b00, 0x04, 0, VSVcode_VDD1, 0x10);	// VDD1_DUT3, Ch2:	ForceA#<15:0> = Value
	vsv_addr = vsv_serial_det(1, 1, 0b00, 0x08, 0, VSVcode_VDD1, 0x10);	// VDD1_DUT4, Ch3:	ForceA#<15:0> = Value
	vsv_addr = vsv_serial_det(1, 1, 0b00, 0x10, 0, VSVcode_VDDQ, 0x20);	// VDDQ_DUT1, Ch4:	ForceA#<15:0> = Value
	vsv_addr = vsv_serial_det(1, 1, 0b00, 0x20, 0, VSVcode_VDDQ, 0x20);	// VDDQ_DUT2, Ch5:	ForceA#<15:0> = Value
	vsv_addr = vsv_serial_det(1, 1, 0b00, 0x40, 0, VSVcode_VDDQ, 0x20);	// VDDQ_DUT3, Ch6:	ForceA#<15:0> = Value
	vsv_addr = vsv_serial_det(1, 1, 0b00, 0x80, 0, VSVcode_VDDQ, 0x20);	// VDDQ_DUT4, Ch7:	ForceA#<15:0> = Value

// #######################################################


	a = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	if (a == 0) {
		FPGA1_Write(0xF0, 0x03);	// LEDs FPGA
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); // MCU LED1
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); // MCU LED2
		a = 1;
	}
	else {
		FPGA1_Write(0xF0, 0x0c);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); // MCU LED1
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // MCU LED2
		a = 0;
	}

	HAL_Delay(100);

//	tgl_b2_test();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_3);
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_HSE, RCC_MCODIV_3);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FMC_NORSRAM_DEVICE;
  hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_ENABLE;
  hsram1.Init.MemoryType = FMC_MEMORY_TYPE_PSRAM;
  hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_8;
  hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_ENABLE;
  hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, fpga_rst_n_Pin|LED3_MCU_Pin|LED4_MCU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, spi1_dac1_cs__Pin|spi1_dac2_cs__Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_DAC1_GPIO_Port, RST_DAC1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_DAC2_GPIO_Port, RST_DAC2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : fm_a_Pin fm_aC1_Pin fm_aC2_Pin fm_aC3_Pin
                           fm_aC4_Pin fm_aC5_Pin fm_aC6_Pin fm_aC7_Pin
                           fm_aC8_Pin */
  GPIO_InitStruct.Pin = fm_a_Pin|fm_aC1_Pin|fm_aC2_Pin|fm_aC3_Pin
                          |fm_aC4_Pin|fm_aC5_Pin|fm_aC6_Pin|fm_aC7_Pin
                          |fm_aC8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : fpga_rst_n_Pin LED3_MCU_Pin LED4_MCU_Pin */
  GPIO_InitStruct.Pin = fpga_rst_n_Pin|LED3_MCU_Pin|LED4_MCU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : spi1_dac1_cs__Pin spi1_dac2_cs__Pin */
  GPIO_InitStruct.Pin = spi1_dac1_cs__Pin|spi1_dac2_cs__Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_DAC1_Pin */
  GPIO_InitStruct.Pin = RST_DAC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(RST_DAC1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_DAC2_Pin */
  GPIO_InitStruct.Pin = RST_DAC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(RST_DAC2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
