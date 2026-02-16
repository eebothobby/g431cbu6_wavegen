/*
 * gwave.h
 *
 *  Created on: May 5, 2025
 *      Author: ashok
 */

#ifndef INC_GWAVE_H_
#define INC_GWAVE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


// To avoid the non-linearity at the extreme range of the DAC, scale the range
// down a bit so it does not go below 45 or above 4045
// #define RES12M 2047
// #define DAC12M 4095
#define RES12M 2000
#define DAC12M 4000
#define DACOFF 45

void initWave(uint8_t wtype, uint16_t  *wave, int nsamples, int nwaves);

#ifdef __cplusplus
}
#endif

#endif /* INC_GWAVE_H_ */
