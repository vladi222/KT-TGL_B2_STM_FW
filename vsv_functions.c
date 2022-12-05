/*	
 *      #########################################################################
 *              Functions  for   VSV command                for KT-4MG+_LB
 *      #########################################################################
*/
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "functions.h"
#include "vsv_functions.h"
//#include "Xo2.h"
 

void FPGA1_Write(uint32_t addr, uint8_t data);
void FPGA2_Write(uint32_t addr, uint8_t data);
uint8_t FPGA1_Read(uint32_t addr);
uint8_t FPGA2_Read(uint32_t addr);


/* ##############
	Reading FPGA ID	(to be moved in functions.c)
#################### */

//#define	FPGA_ID_SIZE	8	// in vsv_functions.h
uint8_t FPGA1_ID[FPGA_ID_SIZE], FPGA2_ID[FPGA_ID_SIZE];

//	id:	1 or 2	-- FPGA U79 or U80
//void	read_fpga_id(int id, uint8_t * FPGA_ID)
void	read_fpga_id(uint8_t * FPGA_ID)
{
//	if (id == 1)
		for (int i = 0; i<FPGA_ID_SIZE; i=i+1)		FPGA_ID[i] = FPGA1_Read(0x80 + i);
//	else if	(id == 2)
//		for (int i = 0; i<FPGA_ID_SIZE; i=i+1)		FPGA_ID[i] = FPGA2_Read(0x80 + i);
}


//	fpga:	1 or 2	-- FPGA U79 or U80
void	readback_vsv(int fpga, uint8_t * vsv_aab, uint8_t * vsv_acd, uint8_t * vsv_bab, uint8_t * vsv_bcd)
{
//	if (fpga == 1)	{
		for (int i = 0; i<8; i=i+1)		vsv_aab[i] = FPGA1_Read(0x50 + i);	// readback from VSVs AA and AB
		for (int i = 0; i<8; i=i+1)		vsv_acd[i] = FPGA1_Read(0x58 + i);
		for (int i = 0; i<8; i=i+1)		vsv_bab[i] = FPGA1_Read(0x60 + i);
		for (int i = 0; i<8; i=i+1)		vsv_bcd[i] = FPGA1_Read(0x68 + i);	// readback from VSVs BC and BD
/*	}
	else if	(fpga == 2)	{
		for (int i = 0; i<8; i=i+1)		vsv_aab[i] = FPGA2_Read(0x50 + i);	// readback from VSVs CA and CB
		for (int i = 0; i<8; i=i+1)		vsv_acd[i] = FPGA2_Read(0x58 + i);
		for (int i = 0; i<8; i=i+1)		vsv_bab[i] = FPGA2_Read(0x60 + i);
		for (int i = 0; i<8; i=i+1)		vsv_bcd[i] = FPGA2_Read(0x68 + i);	// readback from VSVs DC and DD
	}	*/
}

//	uint8_t FPGA1_Read(uint32_t addr)	from fpga_rw.c

//	FPGA LEDs test:
void	fpga_led_test(int id)
{
	  u16 time, lim;
	  u8 test[8], test1[8], test2[8];

	  lim = 5;
//	if (id == 1)	{
		FPGA1_Write(0xf0, 0x00);
		HAL_Delay(100);
		test[0] = FPGA1_Read(0xf0);
		test1[0] = FPGA1_Read(0xf0);
		test2[0] = FPGA1_Read(0xf0);
		for(time=0; time<lim; time++) {};
		for(time=0; time<lim; time++) {};
		FPGA1_Write(0xf0, 0x01);
		HAL_Delay(100);
		test[1] = FPGA1_Read(0xf0);
		test1[1] = FPGA1_Read(0xf0);
		test2[1] = FPGA1_Read(0xf0);
		for(time=0; time<lim; time++) {};
		for(time=0; time<lim; time++) {};
		FPGA1_Write(0xf0, 0x02);
		HAL_Delay(100);
		test[2] = FPGA1_Read(0xf0);
		test1[2] = FPGA1_Read(0xf0);
		test2[2] = FPGA1_Read(0xf0);
		for(time=0; time<lim; time++) {};
		for(time=0; time<lim; time++) {};
		FPGA1_Write(0xf0, 0x03);
		HAL_Delay(100);
		test[3] = FPGA1_Read(0xf0);
		for(time=0; time<lim; time++) {};
		for(time=0; time<lim; time++) {};
		FPGA1_Write(0xf0, 0x04);
		HAL_Delay(100);
		test[4] = FPGA1_Read(0xf0);
//		for(time=0; time<lim; time++) {};
//		for(time=0; time<lim; time++) {};
		FPGA1_Write(0xf0, 0x08);
		HAL_Delay(100);
		test[5] = FPGA1_Read(0xf0);
//		for(time=0; time<lim; time++) {};
//		for(time=0; time<lim; time++) {};
		FPGA1_Write(0xf0, 0x0c);
		test[6] = FPGA1_Read(0xf0);
		FPGA1_Write(0xf0, 0x02);
		test[7] = FPGA1_Read(0xf0);
//	}
//	else	{
//		FPGA2_Write(0xf0, 0x00);	// stop running with this note:		main() at main.c:188 0x8000688

//		HAL_Delay(100);
//		FPGA2_Write(0xf0, 0x01);
//		HAL_Delay(100);
//		FPGA2_Write(0xf0, 0x02);
//		HAL_Delay(100);
//		FPGA2_Write(0xf0, 0x03);
//		HAL_Delay(100);
//		FPGA2_Write(0xf0, 0x04);
//		HAL_Delay(100);
//		FPGA2_Write(0xf0, 0x08);
//		HAL_Delay(100);
//		FPGA2_Write(0xf0, 0x0c);
//	}
}



/* ##############
	All VSV Reset:
		Reset
		FORCE_# = HiZ
		De-assert reset
		Keep	FORCE_# = HiZ	until all channels are programmed.
#################### */
void vsv_reset(void)
{
// Reset active				VSV Reset pin = 1
	FPGA1_Write(ADDR_VSV_RST, 0xFF);	// 0xFF: all 8x VSVs reset active
//	FPGA2_Write(ADDR_VSV_RST, 0xFF);

// All channels	FORCE_# = HiZ	VSV EN pin = 0 	(?)
	FPGA1_Write(ADDR_VSV_EN, 0);		// 0:	 all 8x VSVs 8 channels	FORCE_# = HiZ
//	FPGA2_Write(ADDR_VSV_EN, 0);		//	VSV Enable pin = 0	to all VSV

//	VSV_EN_OUT = VSV_EN:
	FPGA1_Write(ADDR_VSV_EN_OUT, 0);
//	FPGA2_Write(ADDR_VSV_EN_OUT, 0);

	FPGA1_Write(ADDR_VSV_EN_OUT, 1);
//	FPGA2_Write(ADDR_VSV_EN_OUT, 1);

// De-asserting VSV reset pin
	FPGA1_Write(ADDR_VSV_RST, 0);
//	FPGA2_Write(ADDR_VSV_RST, 0);
}


/* ##############
VSV Serial Control	(DS page 58)

	All on-chip DACs and registers are controlled through the CPU serial data port, which is capable of both writing to the chip
	as well as reading back from the chip.

	Address words for every CPU transaction are all 16 bits in length and contain the destination of the data word for the write
	cycle or the source to be read back for a read cycle. Address bits are shifted in LSB first.

	Data words for every CPU transaction are all 16 bits in length and are loaded or read back LSB first, MSB last. The timing
	for data is different for a read cycle vs. a write cycle, as the drivers on the SDIO alternate between going into high
	impedance and driving the line.


	id:	Targeted FPGA -- 1: FPGA U79 (FPGA1);	2:  FPGA U80 (FPGA2)

	rw:		0: read;	1: write	(for d[15:0] only)	(page 58)

	When performing a read transaction, exactly one channel must be selected and programmed to 1, with the remaining channel bits
	all programmed to 0	(page 59)

	addr, d:	From
			TABLE 74. Vesuvius REGISTER MAP		page 60
			Per Pin Registers					page 62-64
			Central Resource Registers			page 65

	target:	selected VSV's	--	0 to 256	(same serial control can go to several VSVs for both read and write -- FPGA function))
					aa	ab	ac	ad	ba	bb	bc	bd		for FPGA1
					ca	cb	...		da	...		dd		for FPGA2

	The read cycle returned data are saved in the FPGA from the address 0x50, three bytes (24 bits) for each VSV chip, but four
	bytes are reserved for each VSV.
		0x50 ... 0x53	for VSV AA of the FPGA1 or CA for FPGA2
		0x54 ... 0x57	for VSV AB of the FPGA1 or CB for FPGA2
		0x58
		0x5C
		0x62 ... 0x65	for BA (DA for FPGA2)
		...

	Depending of the read cycle (page 61), the read-back data may be:
		z	x	d0 ... d15	x	x	x	x	x	x		up to 24 bits
	or
		z	x	x	x	d0 ... d15	x	x	x	x		up to 24 bits

#################### */


uint16_t vsv_serial_addr(uint16_t A15_14, uint16_t CH_NO, uint16_t A5_0)
{
  uint16_t vsv_addr;
	vsv_addr = (A15_14 << 14) + (CH_NO << 6) + A5_0;
	return vsv_addr;
}



//void vsv_serial(uint8_t rw, uint16_t addr, uint16_t d, uint8_t target)
void vsv_serial(int id, uint8_t rw, uint16_t addr, uint16_t d, uint8_t target)
{
  uint8_t addr_l, addr_h, d_l, d_h;
  u8 test[8], test1[8], test2[8], SVSV_STAT0, SVSV_STAT, VSV_EN;

	addr_l = (u8) addr;
	addr_h = (u8) (addr >> 8);
	d_l = (u8) d;
	d_h = (u8) (d >> 8);

//	if (id == 1)	{
		FPGA1_Write(0x20, rw);		//	addr = 0x20:	VSV rw		0: read;	1: write	(for d[15:0] only)
//		test1[0] = FPGA1_Read(0x20);
//		test2[0] = FPGA1_Read(0x20);
		FPGA1_Write(0x21, addr_l);
//		test1[1] = FPGA1_Read(0x21);
//		test2[0] = FPGA1_Read(0x20);
//		test2[1] = FPGA1_Read(0x21);
		FPGA1_Write(0x22, addr_h);
/*		test1[2] = FPGA1_Read(0x22);
		test2[0] = FPGA1_Read(0x20);
		test2[1] = FPGA1_Read(0x21);
		test2[2] = FPGA1_Read(0x22);
*/
		FPGA1_Write(0x23, d_l);
//		test1[3] = FPGA1_Read(0x23);
		FPGA1_Write(0x24, d_h);
//		test1[4] = FPGA1_Read(0x24);

		FPGA1_Write(0x25, target);	//	target = 0 .. 256	-- VSV selection for serial link -- only one VSV or multiple VSV
//		FPGA1_Write(0x25, target);	//	target = 0 .. 256	-- VSV selection for serial link -- only one VSV or multiple VSV
		FPGA1_Write(0x24, d_h);

//		test1[3] = FPGA1_Read(0x23);
//		test1[4] = FPGA1_Read(0x24);
//		test2[5] = FPGA1_Read(0x25);
//		test1[5] = FPGA1_Read(0x25);

//		for (int i = 0; i<8; i=i+1)
//			test[i] = FPGA1_Read(0x20 + i);	// FOR TESTING: READ RW, ADDR, D IN FPGA

	  	SVSV_STAT0 = FPGA1_Read(ADDR_SVSV_STAT);		// 0:	 VSV serial link inactive
		FPGA1_Write(0x40, target);	// 0x40	-- cmd for serial link to VSV(i)	-- target not in use
	  	SVSV_STAT = FPGA1_Read(ADDR_SVSV_STAT);		// 0:	 VSV serial link inactive
	  	VSV_EN = FPGA1_Read(ADDR_VSV_EN);		// 0:	 all 8x VSVs 8 channels	FORCE_# = HiZ
/*	}
	else if	(id == 2)	{
		FPGA2_Write(0x20, rw);
		FPGA2_Write(0x21, addr_l);
		FPGA2_Write(0x22, addr_h);

		FPGA2_Write(0x23, d_l);
		FPGA2_Write(0x24, d_h);

		FPGA2_Write(0x25, target);

		FPGA2_Write(0x40, target);
	}
*/
}


uint16_t vsv_serial_det(int fpga, uint8_t rw, uint16_t A15_14, uint16_t CH_NO, uint16_t A5_0, uint16_t d, uint8_t target)
{
//	fpga = 1, 2				FPGA1 (U79) or FPGA2 (U80)
//	rw = 0 for read
//	Register Structure:		Addr[15:0], Data[15:0]
//	Target:		selected VSV's	--	0 to 256	(same serial control can go to several VSVs for write -- FPGA function)

  uint16_t vsv_addr;

  	  vsv_addr = (A15_14 << 14) + (CH_NO << 6) + A5_0;
  	  vsv_serial(fpga, rw, vsv_addr, d, target);
  	  return vsv_addr;
}


/* ##############
	VSV Serial Control Examples:

		VSVS Die ID	--	Read Only	Central Resource Registers		(VSVS DS Page 65)
				A15	A14	A[13:7]	A[6:0]	D[15:0]
				1	1	0 0...0	127		Die ID
									Page 59:	D[15:4]: Product ID (xF10);	D[3:0]: Die Revision (2)

					Ex: selected all VSV devices:	ab to bd		0xFF

			vsv_serial(1, 0, 0xC07F, 0x00, 0xFF);		// to FPGA1	U79
			vsv_serial(2, 0, 0xC07F, 0x00, 0xFF);		// to FPGA2	U80
		The read data in the FPGA registers	(one 24-bit register for each Vesuvius device; 32-bit register reserved)
			AA_VD[15:0], AB_VD[15:0], ... BD[15:0]
			(FPGA addr:	ADDR_RVSV_AA, ADDR_RVSV_AB, ... ADDR_RVSV_BD)
constant ADDR_RVSV_AA	: std_logic_vector(MEM_ADDR_WIDTH-1 downto 0):= 10x"050";
constant ADDR_RVSV_AB	: std_logic_vector(MEM_ADDR_WIDTH-1 downto 0):= 10x"054";
...



		Programing Vesuvius internal connection		Force to Sense		bit Con-FS# = 1	(VSVS DS page 40, 47)
			Diagnostics and Calibration Register	(page 62)	Force to Sense: bit D8 (Con-FS#) = 1; WE on: D10=1
				A15		A14		A[13:6]					A[5:0]		D[15:0]
				1		0		selected channels		000100		0x0500

					ex.:	selected channels:	4 and 0	--	A[13:6] = 00010001
						selected VSV devices:	ab and bc		0x42

				vsv_serial(1, 1, 0x8444, 0x500, 0x42);	// write to FPGA1	U79
#################### */



//	DPS Control/Miscellaneous:	(page 62)	A15_14 = 0b10; A[5:0] = 1
//		vsv_address = vsv_serial_det(fpga, rw, 0b10, CH_NO, 1, data, target);
//						D15=WE	D[14:6]=Con-Res/Cap#
//						D5=WE	D4=FV-Mode#
//									FV-Mode#=0 -> High Current (uses VCC0_#)
//									FV-Mode#=1 -> Low Current (uses VCC)
//						D3=WE,	D2=Sel-DPS-En#,		D1=Sel-RT-En#,	D0=CPU-En#
/*
 	Page 33:

	Sel-RT-En#	Sel-DPS-En#		DPS-En#
	----------------------------------------
	0			1				CPU-En#
	1			1				EN
	X			0				RT-En#•Chan-Alarm#
*/
void vsv_init(void)		// so far, FPGA2 only
{
	  uint16_t vsv_addr;

	vsv_addr = vsv_serial_det(2, 1, 0b10, 0xff, 1, 0x0020, 0xFF);		// all channels, all VSVs	FV-Mode# = 0
	vsv_addr = vsv_serial_det(2, 1, 0b10, 0x22, 1, 0x000C, 0xFF);		// Channels 1/5, all VSVs	HiZ
//	??	From -4V to -3.7V:
	vsv_addr = vsv_serial_det(2, 1, 0b10, 0xBB, 1, 0x000D, 0xFF);		// Channels 0/2/3/4/6/7, all VSVs	CPU-En# = 1

//	Diagnostics and Calibration Register	page 62
	//		Addr[13:6]:	selected channels -- may be all channels for programming, only one for reading
	//		Programming Vesuvius internal Force to Sense connection		D8=Con-FS#= 1; D10=WE=1		VSVS DS page 47, 62
	//
	//		D15, D10, D4:		WE bits		page 58			set all to 1 to allow updating all data bits
	//			D10 allows updating		D8=Con-FS#
	//	D[15:0]:			1 x x x		x 1 x 1		x x x 1		x x x x		0x8510	all WE bits = 1
	//	D[15:0] read back	x x x x		x x x 1		x x x x		x x x x		(likely 0x0100)
	// Diagnostics	page 53		D[3:0]
	//
	//	Addr[15:0]:			1 0 x x		x x x x		x x 0 0		0 1 0 0			Channels selection [7:0]:	A13 ... A6

	vsv_serial(2, 1, 0xBFC4, 0x8510, 0xff);	// all ch's, all VSV's	--	Diagnostics and Calibration -- Force to Sense connection

	vsv_addr = vsv_serial_det(2, 1, 0b10, 0xFF, 5, 0x1800, 0xFF);		// All Chs; IR = IR5 (512mA Range)	-- All VSV

	vsv_addr = vsv_serial_det(2, 1, 0b10, 0xFF, 0, 0x400, 0xFF);	// All Chs	Set FVR0=0 (8V Range)	-- All VSV

//		vsv_serial_det(fpga, rw, A15_14, CH_NO, A5_0, data, target)
	vsv_addr = vsv_serial_det(2, 1, 0b00, 0xFF, 0x10, 0x7FFF, 0xFF);	// All chs	ForceA#<15:0> Offset = 0V	-- All VSV

	vsv_addr = vsv_serial_det(2, 1, 0b00, 0xff, 0x20, 0x7FFF, 0xFF);	// All chs	ForceA#<15:0> Gain = 1.0	-- All VSV

	vsv_addr = vsv_serial_det(2, 1, 0b10, 0xFF, 1, 0xD, 0xFF);	// All Chs; CPU-EN0 = 1		(Channel Enabled)	-- All VSV		D3=WE,	D2=Sel-DPS-En#,		D1=Sel-RT-En#,	D0=CPU-En#
}



