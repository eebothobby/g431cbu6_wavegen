/*
 * gwave.c
 *
 *  Created on: May 5, 2025
 *      Author: ashok
 */

#include "gwave.h"
#include <math.h>


// return the uint16_t of absolute value of a float
uint16_t fiabs(float v) {
	return v > 0.0 ? (uint16_t) v : (uint16_t) -v;
}

double sinc(double x) {
    if (x == 0.0) {
        return 1.0;
    }
    return sin(M_PI * x) / (M_PI * x);
}

void initWave(uint8_t wtype, uint16_t  *wave, int nsamples, int nwaves) {
	float phase;

	for (int i = 0; i < nsamples; i++) {
		phase = (i + 0.5) / nsamples;  // Normalize phase [0, 1)
		switch (wtype) {
		case 0: // sin
			wave[i] = ((sin(phase * 2 * M_PI) + 1) * RES12M) + DACOFF;
			break;
		case 1: // rampup
			wave[i] = (uint16_t)(phase * DAC12M) + DACOFF;
			break;
		case 2: // rampdown
			wave[i] = (uint16_t) (DAC12M * (1.0 - phase)) + DACOFF;
			break;
		case 3: // triangle
			wave[i] = fiabs(DAC12M * (1.0 - 2.0 * phase)) + DACOFF;
			break;
		case 4: // sinc
			// phase*30, center on phase * 15, minimum is around -0.218, max is 1.0
			wave[i] = ((sinc(15.0 - 30.0 * phase) + 0.218)/1.218 * DAC12M) + DACOFF;
			break;
		}
	}
	// duplicate the wave[0..nsamples-1] (nwaves - 1) times
	for (int nw = 1; nw < nwaves; nw++) {
		for (int i = 0; i < nsamples; i++) {
			wave[(nsamples * nw) + i] = wave[i];
		}
	}
}
