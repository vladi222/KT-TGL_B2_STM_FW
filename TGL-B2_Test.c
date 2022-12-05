/**
  ******************************************************************************
  * @file           : TGL-B2_Test.c	221128	1:08 PM
  * @brief          : Testing program for TGL B2
  ******************************************************************************
*/

#include <stdlib.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "vsv_functions.h"
#include "functions.h"
#include "boardspecs.h"

void AD5592_Init(void);
void SetVDD_4671(u16 vout, u8 id);		// id for DAC dev.	0 to 3
void SetVPP_4671(u16 vout, u8 id);		// id for DAC dev.	0 to 3
void SetVDDQ_4671(u16 vout, u8 id);		// id for DAC dev.	0 to 3	Group C, D, B, A
HAL_StatusTypeDef DAC_WriteReg(u16 value, int id);
uint16_t DAC_ReadReg(int id);
HAL_StatusTypeDef DAC_WriteReg_8SPI(u16 value, int id);
HAL_StatusTypeDef DAC_ReadReg_8SPI(u16 * value, int id);
uint16_t vsv_serial_det(int fpga, uint8_t rw, uint16_t A15_14, uint16_t CH_NO, uint16_t A5_0, uint16_t d, uint8_t target);
void vsv_reset(void);

void FPGA1_Write(uint32_t addr, uint8_t data);
uint8_t FPGA1_Read(uint32_t addr);

void read_fpga_id(uint8_t * FPGA_ID);
void fpga_reset(void);

void vsv_serial(int id, uint8_t rw, uint16_t addr, uint16_t d, uint8_t target);
void vsv_init(void);

HAL_StatusTypeDef DAC_WriteReg_16SPI(u16 value, int id);	// for 16-bit SPI


void tgl_b2_test(void)
{

//	KT-TGL B2	###############################

	  uint8_t dac_io;

//		uint16_t status[5];
//		uint8_t MCLK_Count;
		int a, b, c;
		uint16_t vsv_addr;
		uint16_t value;
		uint16_t DACcode, DACcode0;
		uint16_t rr_value, r_value;
		HAL_StatusTypeDef hstatus;
		HAL_StatusTypeDef status5592;
		HAL_StatusTypeDef statusFPGA79, statusFPGA80;

		uint32_t addr, three_bytes[5], error[5];
		uint32_t two_bytes[5], rvalue[5];
		uint16_t chno, chdata;

	  	uint16_t pTxData;
	  	uint16_t pRxData;
	  	u16 nss_pin[2] = {DAC1_NSS, DAC2_NSS,};       // u12, u13

		uint8_t FPGA_ID[8];
		uint8_t F1_RST, F1_EN, F1_OT, SVSV_STAT0, SVSV_STAT;
		uint8_t F1_VSV_ID[8];
		uint8_t F1_AAB[8], F1_ACD[8], F1_BAB[8], F1_BCD[8];
		uint8_t F1_AAB_ID[8];

		uint32_t Step_Wait;
//
// #########################################################
//
// 	Start Board Test:
//
// #########################################################
//							  bit[15:12]		DAC U12 (DAC ID = 0)	DAC U13 (DAC ID = 1)
//	dac_io = 0;	// for DAC IO0	1 000	U27	U30		DAC_DD2_1 - R186;	DAC_DD2_3 - R205		Top
//	dac_io = 1;	// for DAC IO1	1 001	U28	U31		DAC_DD2_2 - R188;	DAC_DD2_4 - R207		Bottom
//	dac_io = 2;	// for DAC IO2	1 010	U26	U29		DAC_IN12 - R176;	DAC_IN34 - R195			Top
//	dac_io = 3;	// for DAC IO3	1 011	U22	U23		DAC_DD1 - R154;		DAC_DDQ - R157 (not pop -- DAC not connected)	Bottom
//
//
//		value = 0x8000 | (dac_io*0x1000) | (u16)DACcode;	// DACcode 12-bit: 0 to 0xFFF
//		DAC_WriteReg_16SPI(value, dac_ID);					// dac_ID = 0 or 1	--	U12 or U13
//
// Board Top:		L3, L4, L5 side
// Board Bottom:	VSV, 4651, L6, L7 side
//
//##############################################################
//
//	Start AD5592 Testing
//
//		For this section, use SPI setting	High, 1Edge
//
//##############################################################

//	Writing to the DAC readback register	page 30		to allow reading the input register of each DAC
//		D[15:5]=	0 000 1 000 000
//		D[4:3]=		11: readback enabled;	00: readback disabled (default)
//		D[2:0]		Select DAC channel
//
//	  		value = 0x0818;		//	allow reading input register of DAC0

	Step_Wait = 50;

	AD5592_Init();

	DACcode0 = 0x200;

	for(int i=0; i<9; i++)
	{
		DACcode = DACcode0*i;
		if (DACcode > 0xFFF)	{DACcode = 0xFFF;}
		value = 0x8000 | (2*0x1000) | DACcode;		// U26, U29
		DAC_WriteReg_16SPI(value, 0);				// U26		VIN12		(12V)	TP: L6 Bottom; C292 Top
		DAC_WriteReg_16SPI(value, 1);				// U29		VIN34		(12V)	TP: L7 Bottom; C321 Top
		value = 0x8000 | (3*0x1000) | DACcode;		// U22, U23
		DAC_WriteReg_16SPI(value, 0);				// U22		VIN_DD1		(12V)	TP: D2/3/4/5 Bottom
		DAC_WriteReg_16SPI(value, 1);				// U23		VIN_DDQ		(12V)	TP: D6/7/8/9 Bottom
brkpoint1:	HAL_Delay(Step_Wait);
	}

// Max V for VIN12, VIN34, VIN_DD1, VIN__DDQ	(DACcode = 0)	for the next measurements:
	value = 0x8000 | (2*0x1000);				// U26, U29
	DAC_WriteReg_16SPI(value, 0);				// U26		VIN12		(12V)	TP: L6 Bottom; C292 Top
	DAC_WriteReg_16SPI(value, 1);				// U29		VIN34		(12V)	TP: L7 Bottom; C321 Top
	value = 0x8000 | (3*0x1000);				// U22, U23
	DAC_WriteReg_16SPI(value, 0);				// U22		VIN_DD1		(12V)	TP: D2/3/4/5 Bottom
	DAC_WriteReg_16SPI(value, 1);				// U23		VIN_DDQ		(12V)	TP: D6/7/8/9 Bottom

	for(int i=0; i<9; i++)
	{
		DACcode = DACcode0*i;
		if (DACcode > 0xFFF)	{DACcode = 0xFFF;}
		value = 0x8000 | (0*0x1000) | DACcode;		// U27, U30
		DAC_WriteReg_16SPI(value, 0);				// U27		VDD2_DUT1	(VIN12)	TP: C316 Bottom; R181.2 Top
		DAC_WriteReg_16SPI(value, 1);				// U30		VDD2_DUT3	(VIN34)	TP: C346 Bottom; R200.2 Top
		value = 0x8000 | (1*0x1000) | DACcode;		// U28, U31
		DAC_WriteReg_16SPI(value, 0);				// U28		VDD2_DUT2	(VIN12)	TP: C317, R183 Bottom
		DAC_WriteReg_16SPI(value, 1);				// U31		VDD2_DUT4	(VIN34)	TP: C335 Bottom
brkpoint2:	HAL_Delay(Step_Wait);
	}

//##############################################################
//
//	End AD5592 Testing
//
//##############################################################


//##############################################################
//
//	Start Vesuvius Testing
//
//##############################################################

//	uint16_t vsv_serial_det(int fpga, uint8_t rw, uint16_t A15_14, uint16_t CH_NO, uint16_t A5_0, uint16_t data, uint8_t target)
//								fpga and target	ignored for TGL	--	only one VSV and one FPGA

//#########################################################
//	Read VSV Die ID:		page 65		Addr = 0xC07F
//
//		read-back:	Page 59		D[15:4]: Product ID (0xF10);	D[3:0]: Die Revision (0x2)
//	Expected:
// 		svsv_r(J)[23:0]:	xxxx xx	11	1100 01 00		00	00 10 xx
//				D[15:4]:			11	1100 01 00		00				0xF10
//				D[3:0]:										00 10		0x2
//
//	Typical return:		0xb		0xc4		0b xxxx xx11	(0xB	0xC4	0xB or 0x7 or 0x3, etc.)
//					0000 1011	1100 0100	xxxx xx11
//
//	F1_VSV_ID =		{addr(0x50), addr(0x51), addr(0x52)};
//	Typical return:		0xb			0xc4	0b xxxx xx11
//					0000 1011	1100 0100	xxxx xx11
//
//	{addr(0x52), 	addr(0x51), 	addr(0x50):
//	xxxx xx 11		11 00 01 00		00 00 10	11
//
//#########################################################
//
// #########################################################
//
// 	-- Vesuvius Chip ID -- Die ID page 65:
//			Addr[15:0]:	0xC07F		1 1 0 0		0 0 0 0		0 1 1 1		1 1 1 1
//
// #########################################################
//		Read VSV Die ID:
// #########################################################

	vsv_serial(1, 1, 0xC000, 0, 0xFF);			//	Software Reset	All VSV

	FPGA1_Write(ADDR_VSV_RST, 0x0);				// Reset pin = 0 (vsv_rst pin) inactive
	FPGA1_Write(ADDR_VSV_EN, 1);				// EN pin = 1, if S/W selected, FORCE_# are enabled	(0: all 8 channels FORCE_# = HiZ)
	F1_RST = FPGA1_Read(ADDR_VSV_RST);			// 0xFF: all 8x VSVs reset active
	F1_EN = FPGA1_Read(ADDR_VSV_EN);			// 0:	 all 8x VSVs 8 channels	FORCE_# = HiZ
	SVSV_STAT0 = FPGA1_Read(ADDR_SVSV_STAT);	// 0:	 VSV serial link inactive

	vsv_serial(1, 0, 0xC07F, 0x00, 0xFF);									// to FPGA1 -- read VSV Die ID
	SVSV_STAT = FPGA1_Read(ADDR_SVSV_STAT);									// 0:	 VSV serial link inactive?
	for (int i = 0; i<8; i=i+1)		F1_AAB_ID[i] = FPGA1_Read(0x20 + i);	// 00 7f c0 00 00
	for (int i = 0; i<8; i=i+1)		F1_VSV_ID[i] = FPGA1_Read(0x50 + i);	// 0b c4 07		VSV Die ID
	F1_RST = FPGA1_Read(ADDR_VSV_RST);										// 0xFF: all 8x VSVs reset active
//

// #########################################################
//		Programming VSV FORCE_# outputs:
// #########################################################
// #########################################################
//
//	Measurement Unit Source	Selection:	A15_14=0b10, A[5:0]=5	(page 62)
	vsv_addr = vsv_serial_det(1, 1, 0b10, 0xFF, 5, 0x1800, 0xFF);	// All Chs; All VSV; IR = IR5 (512mA Range)	-- D12=WE=1, D11=IR#<5>=1

	vsv_addr = vsv_serial_det(1, 1, 0b10, 0xFF, 0, 0x1800, 0xFF);	// All Chs; All VSV; FVR# = 0x00 (8V Range)	-- D10=WE=1, D9=0
	vsv_addr = vsv_serial_det(1, 1, 0b10, 0xFF, 4, 0x500, 0xFF);	// All chs, all VSV -- FORCE_# to SENSE_#
	vsv_addr = vsv_serial_det(1, 1, 0b10, 0xFF, 1, 0xD, 0xFF);		// All Chs Active -- D3=WE=1, D2=Sel-DPS-En#=1, D1=Sel-RT-En=0#, D0=CPU-En#=DSP-En#=1
	for (int i = 0; i<13; i=i+1)
	{
		value = 0x4000 + i*0x800;
		vsv_addr = vsv_serial_det(1, 1, 0b00, 0x01, 0, value, 0x10);	// VDD1_DUT1, Ch0:	ForceA#<15:0> = Value
		vsv_addr = vsv_serial_det(1, 1, 0b00, 0x02, 0, value, 0x10);	// VDD1_DUT2, Ch1:	ForceA#<15:0> = Value
		vsv_addr = vsv_serial_det(1, 1, 0b00, 0x04, 0, value, 0x10);	// VDD1_DUT3, Ch2:	ForceA#<15:0> = Value
		vsv_addr = vsv_serial_det(1, 1, 0b00, 0x08, 0, value, 0x10);	// VDD1_DUT4, Ch3:	ForceA#<15:0> = Value
		vsv_addr = vsv_serial_det(1, 1, 0b00, 0x10, 0, value, 0x20);	// VDDQ_DUT1, Ch4:	ForceA#<15:0> = Value
		vsv_addr = vsv_serial_det(1, 1, 0b00, 0x20, 0, value, 0x20);	// VDDQ_DUT2, Ch5:	ForceA#<15:0> = Value
		vsv_addr = vsv_serial_det(1, 1, 0b00, 0x40, 0, value, 0x20);	// VDDQ_DUT3, Ch6:	ForceA#<15:0> = Value
		vsv_addr = vsv_serial_det(1, 1, 0b00, 0x80, 0, value, 0x20);	// VDDQ_DUT4, Ch7:	ForceA#<15:0> = Value
brkpoint3:	HAL_Delay(Step_Wait);
	}

//##############################################################
//
//	End Vesuvius Testing
//
//##############################################################


// #########################################################
//
// 	End Board Test
//
// #########################################################
}
