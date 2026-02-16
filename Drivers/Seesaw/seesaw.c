/*
 * * Library for a Adafruit Seesaw device.
 * This code is adapted from https://github.com/adafruit/Adafruit_Seesaw
* License: BSD
 */

#include "seesaw.h"

static int seesaw_read(uint8_t regHigh, uint8_t regLow, uint8_t *buf, uint8_t num, seesaw_t *ss) {
	uint16_t memaddr = (regHigh << 8) + regLow;
	if (HAL_I2C_Mem_Read(ss->i2ch, ss->i2caddr, memaddr, 2, buf, num, 1000) != HAL_OK) {
		return 0;
	}
	return 1;
}

static int seesaw_write(uint8_t regHigh, uint8_t regLow, uint8_t *buf, int8_t num, seesaw_t *ss) {
	uint16_t memaddr = (regHigh << 8) + regLow;
	if (HAL_I2C_Mem_Write(ss->i2ch, ss->i2caddr, memaddr, 2, buf, num, 1000) != HAL_OK) {
		return 0;
	}
	return 1;
}

static int seesaw_write8(uint8_t regHigh, uint8_t regLow, uint8_t val, seesaw_t *ss) {
	uint16_t memaddr = (regHigh << 8) + regLow;
	if (HAL_I2C_Mem_Write(ss->i2ch, ss->i2caddr, memaddr, 2, &val, 1, 1000) != HAL_OK) {
		return 0;
	}
	return 1;
}

// i2caddr need only be the three address bits for the part (0 .. 7).
// returns 1 on success, 0 on failure
int seesaw_Begin(I2C_HandleTypeDef *hi2c,  int i2caddr, seesaw_t *ss) {
	uint8_t c;

	ss->i2ch = hi2c;
	ss->i2caddr = (SEESAW_I2CADDR | (i2caddr & 0x7)) << 1;
	if (HAL_I2C_IsDeviceReady(hi2c, ss->i2caddr, 1, 10) != HAL_OK) {
		return 0;
	}

	if (!seesaw_read(SEESAW_STATUS_BASE, SEESAW_STATUS_HW_ID, &c, 1, ss)) {
		return 0;
	}
	if ((c == SEESAW_HW_ID_CODE_SAMD09) || (c == SEESAW_HW_ID_CODE_TINY817) ||
		(c == SEESAW_HW_ID_CODE_TINY807) || (c == SEESAW_HW_ID_CODE_TINY816) ||
		(c == SEESAW_HW_ID_CODE_TINY806) || (c == SEESAW_HW_ID_CODE_TINY1616) ||
		(c == SEESAW_HW_ID_CODE_TINY1617)) {
	  	  ss->hwtype = c;
	  	  return 1;
	}
	return 0;
}

int seesaw_SWReset(seesaw_t *ss) {
	return (seesaw_write8(SEESAW_STATUS_BASE, SEESAW_STATUS_SWRST, 0xFF, ss));
}

uint32_t seesaw_getOptions(seesaw_t *ss) {
	uint8_t buf[4];
	uint32_t ret;

	seesaw_read(SEESAW_STATUS_BASE, SEESAW_STATUS_OPTIONS, buf, 4, ss);
	ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
	                 ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
	return ret;
}

int32_t seesaw_getEncoderPos(seesaw_t *ss, uint8_t encoder) {
	uint8_t buf[4];
	seesaw_read(SEESAW_ENCODER_BASE, SEESAW_ENCODER_POSITION + encoder, buf, 4, ss);
	int32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
	                ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];

	return ret;
}

int32_t seesaw_getEncoderDelta(seesaw_t *ss, uint8_t encoder) {
	uint8_t buf[4];
	seesaw_read(SEESAW_ENCODER_BASE, SEESAW_ENCODER_DELTA + encoder, buf, 4, ss);
	int32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
	                ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];

	return ret;
}

// set the mode of multiple GPIO pins at once.
// pins : a bitmask of the pins to write. On the SAMD09 breakout,
//        this corresponds to the number on the silkscreen.
//		  For example, passing 0b0110 will set the mode of pins 2 and 3.
// mode: The mode to set the pins to. One of INPUT, OUTPUT, or INPUT_PULLUP

int seesaw_pinModeBulk(uint32_t pins, uint8_t mode, seesaw_t *ss) {
  uint8_t cmd[] = {(uint8_t)(pins >> 24), (uint8_t)(pins >> 16),
                   (uint8_t)(pins >> 8), (uint8_t)pins};
  switch (mode) {
  case OUTPUT:
    return(seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRSET_BULK, cmd, 4, ss));
    break;
  case INPUT:
    return(seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, cmd, 4, ss));
    break;
  case INPUT_PULLUP:
    if (!seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, cmd, 4, ss)) {
    	return 0;
    }
    if (!seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENSET, cmd, 4, ss)) {
    	return 0;
    }
    if (!seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_SET, cmd, 4, ss)) {
    	return 0;
    }
    break;
  case INPUT_PULLDOWN:
    if (!seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, cmd, 4, ss)) {
    	return 0;
    }
    if (!seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENSET, cmd, 4, ss)) {
    	return 0;
    }
    if (!seesaw_write(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_CLR, cmd, 4, ss)) {
    	return 0;
    }
    break;
  }
  return 1;
}

int seesaw_pinMode(uint8_t pin, uint8_t mode, seesaw_t *ss) {
    return (seesaw_pinModeBulk(1ul << pin, mode, ss));
}

int seesaw_enableEncoderInterrupt(seesaw_t *ss, uint8_t encoder) {
  return (seesaw_write8(SEESAW_ENCODER_BASE, SEESAW_ENCODER_INTENSET + encoder, 0x01, ss));
}

int seesaw_setGPIOInterrupts(uint32_t pins, int enabled, seesaw_t *ss) {
  uint8_t cmd[] = {(uint8_t)(pins >> 24), (uint8_t)(pins >> 16),
                   (uint8_t)(pins >> 8), (uint8_t)pins};
  uint8_t setclr = enabled? SEESAW_GPIO_INTENSET : SEESAW_GPIO_INTENCLR;
  return (seesaw_write(SEESAW_GPIO_BASE, setclr, cmd, 4, ss));
}

uint16_t seesaw_analogRead(uint8_t pin, seesaw_t *ss) {
  uint8_t buf[2];
  uint8_t p = 0;

  if (ss->hwtype == SEESAW_HW_ID_CODE_SAMD09) {
    switch (pin) {
    case ADC_INPUT_0_PIN:
      p = 0;
      break;
    case ADC_INPUT_1_PIN:
      p = 1;
      break;
    case ADC_INPUT_2_PIN:
      p = 2;
      break;
    case ADC_INPUT_3_PIN:
      p = 3;
      break;
    default:
      return 0;
    }
  } else if ((ss->hwtype == SEESAW_HW_ID_CODE_TINY807) ||
             (ss->hwtype == SEESAW_HW_ID_CODE_TINY817) ||
             (ss->hwtype == SEESAW_HW_ID_CODE_TINY816) ||
             (ss->hwtype == SEESAW_HW_ID_CODE_TINY806) ||
             (ss->hwtype == SEESAW_HW_ID_CODE_TINY1616) ||
             (ss->hwtype == SEESAW_HW_ID_CODE_TINY1617)) {
    p = pin;
  } else {
    return 0;
  }

  seesaw_read(SEESAW_ADC_BASE, SEESAW_ADC_CHANNEL_OFFSET + p, buf, 2, ss);
  uint16_t ret = ((uint16_t)buf[0] << 8) | buf[1];
  return ret;
}

int seesaw_digitalRead(uint8_t pin, seesaw_t *ss) {
   if (seesaw_digitalReadBulk((1ul << pin), ss) != 0) {
	   return 1;
   } else {
	   return 0;
   }
}

uint32_t seesaw_digitalReadBulk(uint32_t pins, seesaw_t *ss) {
  uint8_t buf[4];
  seesaw_read(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK, buf, 4, ss);
  uint32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                 ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
  return ret & pins;
}
