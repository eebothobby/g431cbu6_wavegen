/*
 * mcp45hvx1.h
 *
 * Library for MC45HVX1 high voltage digital potentiometer.
 * Loosely based on code by Jonathan Dempsey JDWifWaf@gmail.com
 *  Created on: Feb 20, 2025
 *      Author: ashok
 */

#ifndef __MCP45HVX1_H
#define ___MCP45HVX1_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

 // Base I2C address
 #define MCP45HVX1_I2CADDR 0x3C

typedef struct {
  	I2C_HandleTypeDef *i2ch;
  	int i2caddr;
} mcp45hvx1_t;

// i2caddr need only be the 2 address bits for the part (0 .. 3).
// returns 1 on success, 0 on failure
int mcp45hvx1_Begin(I2C_HandleTypeDef *hi2c,  int i2caddr, mcp45hvx1_t *dpot);

// Set wiper value.
// Return 1 on success, 0 on failure
int mcp45hvx1_setWiper(uint8_t wiperVal, mcp45hvx1_t *dpot);

#ifdef __cplusplus
}
#endif

#endif // ifndef __MCP45HVX1_H
