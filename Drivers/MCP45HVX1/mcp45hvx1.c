/*
 * mcp45hvx1.c
 * Library for MC45HVX1 high voltage digital potentiometer.
 * Loosely based on code by Jonathan Dempsey JDWifWaf@gmail.com
 *  Created on: Feb 20, 2025
 *      Author: ashok
 */

#include "mcp45hvx1.h"


/* TCON configuration.................. */
#define TCON_R0HW (0x08)  // Shutdown Resistor Force
#define TCON_R0A  (0x04)  // Terminal A Connection
#define TCON_R0W  (0x02)  // Wiper Connection
#define TCON_R0B  (0x01)  // Terminal B Connection

#define GCALL_TCON          (0x60)
#define GCALL_WIPER         (0x40)
#define GCALL_WIPERUP       (0x42)
#define GCALL_WIPERDWN      (0x44)
#define GCALL_COM_WRITE     (0x02)
#define GCALL_COM_RWRITE    (0x03)
#define GCALL_COM_WIPERINC  (0x42)
#define GCALL_COM_WIPERDEC  (0x44)

#define MEM_WIPER           (0x00)
#define MEM_TCON            (0x40)

#define COM_WRITE           (0x00)
#define COM_READ            (0x0C)
#define COM_WIPERINC        (0x04)
#define COM_WIPERDEC        (0x08)

// i2caddr need only be the 2 address bits for the part (0 .. 3).
// returns 1 on success, 0 on failure
int mcp45hvx1_Begin(I2C_HandleTypeDef *hi2c,  int i2caddr, mcp45hvx1_t *dpot) {
	dpot->i2ch = hi2c;
	dpot->i2caddr = (MCP45HVX1_I2CADDR | (i2caddr & 0x3)) << 1;
	if (HAL_I2C_IsDeviceReady(hi2c, dpot->i2caddr, 1, 10) != HAL_OK) {
		return 0;
	}
	return 1;
}

// Set wiper value.
// Return 1 on success, 0 on failure
int mcp45hvx1_setWiper(uint8_t wiperVal, mcp45hvx1_t *dpot) {
	uint8_t buf[2];
	I2C_HandleTypeDef *hi2c;
	int i2caddr;

	hi2c = dpot->i2ch;
	i2caddr = dpot->i2caddr;


	buf[0] = MEM_WIPER | COM_WRITE;
	buf[1] = wiperVal;
	if (HAL_I2C_Master_Transmit(hi2c, i2caddr, buf, 2, 100) != HAL_OK) {
		return 0;
	}
	return 1;
}
