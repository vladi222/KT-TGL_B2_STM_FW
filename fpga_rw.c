#include "stdint.h"
#define FPGA1   ((uint32_t)0x60000000)
#define FPGA2   ((uint32_t)0x60000000)
//#define FPGA2   ((uint32_t)0x64000000)

/*	from FPGA:

LOCATE COMP "rst_n_0" SITE "L3" ;	# -- from MCU PA0 WKUP - same signal for both FPGAs	-- so far, not use for MCU wakeup


	if rising_edge(clk) then
		if (rst_n = '1') then
--			led_reg <= "01101001";	-- LED3,4 pulsing with different frequencies
			if ((FMC_NE = '0') and (FMC_NWE = '0') and (FMC_NOE = '1'))	then		-- MCU writing
				if (MEM_ADDR = ADDR_LED)	then   		-- REG_ADDR = x"80"
					led_reg <= MEM_WD(7 downto 0);
				elsif	(MEM_ADDR = (REG_ADDR + 7))	then
					data_in <= MEM_WD;

		...

			elsif	((FMC_NE = '0') and (FMC_NOE = '0'))	then		-- MCU reading	?and (FMC_NWE = '1')?
				if	(MEM_ADDR = REG_ADDR)	then
					MEM_RD <= FPGA_REV;
				elsif	(MEM_ADDR = (REG_ADDR + 1))	then
					MEM_RD <= YEAR;

		...
*/

/*
From "functions.c:

void fpga_reset(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // MCU PA0/WKUP
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // MCU PA0/WKUP
	HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // MCU PA0/WKUP
}

*/

//--------------------------------------------------------------
// Sends 8-bit value to FPGA
//--------------------------------------------------------------
void FPGA1_Write(uint32_t addr, uint8_t data)
{
  *(uint8_t *) (FPGA1 + addr) = data;

//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // MCU PA0/WKUP	-- FPGA rst_n input signal

}

void FPGA2_Write(uint32_t addr, uint8_t data)
{
  *(uint8_t *) (FPGA2 + addr) = data;
}

//--------------------------------------------------------------
// Reads 8-bit value from FPGA
//--------------------------------------------------------------
uint8_t FPGA1_Read(uint32_t addr)
{
  uint8_t data = 0;
  data = *(volatile uint8_t*) (FPGA1 + addr);
  return data;
}

uint8_t FPGA2_Read(uint32_t addr)
{
  uint8_t data = 0;
  data = *(volatile uint8_t*) (FPGA2 + addr);
  return data;
}
