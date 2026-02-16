/*******************************************************************************
* Library for a Adafruit Seesaw device.
* This code is adapted from https://github.com/adafruit/Adafruit_Seesaw
* License: BSD
******************************************************************************
*/
#ifndef __SEESAW_H
#define __SEESAW_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include "main.h"

// Base I2C address
#define SEESAW_I2CADDR 0x4B


 /** Module Base Addreses
  *  The module base addresses for different seesaw modules.
  */
 enum {
   SEESAW_STATUS_BASE = 0x00,
   SEESAW_GPIO_BASE = 0x01,
   SEESAW_SERCOM0_BASE = 0x02,

   SEESAW_TIMER_BASE = 0x08,
   SEESAW_ADC_BASE = 0x09,
   SEESAW_DAC_BASE = 0x0A,
   SEESAW_INTERRUPT_BASE = 0x0B,
   SEESAW_DAP_BASE = 0x0C,
   SEESAW_EEPROM_BASE = 0x0D,
   SEESAW_NEOPIXEL_BASE = 0x0E,
   SEESAW_TOUCH_BASE = 0x0F,
   SEESAW_KEYPAD_BASE = 0x10,
   SEESAW_ENCODER_BASE = 0x11,
   SEESAW_SPECTRUM_BASE = 0x12,
 };

 /** GPIO module function address registers
  */
 enum {
   SEESAW_GPIO_DIRSET_BULK = 0x02,
   SEESAW_GPIO_DIRCLR_BULK = 0x03,
   SEESAW_GPIO_BULK = 0x04,
   SEESAW_GPIO_BULK_SET = 0x05,
   SEESAW_GPIO_BULK_CLR = 0x06,
   SEESAW_GPIO_BULK_TOGGLE = 0x07,
   SEESAW_GPIO_INTENSET = 0x08,
   SEESAW_GPIO_INTENCLR = 0x09,
   SEESAW_GPIO_INTFLAG = 0x0A,
   SEESAW_GPIO_PULLENSET = 0x0B,
   SEESAW_GPIO_PULLENCLR = 0x0C,
 };


 /** status module function address registers
  */
 enum {
   SEESAW_STATUS_HW_ID = 0x01,
   SEESAW_STATUS_VERSION = 0x02,
   SEESAW_STATUS_OPTIONS = 0x03,
   SEESAW_STATUS_TEMP = 0x04,
   SEESAW_STATUS_SWRST = 0x7F,
 };

 /** timer module function address registers
  */
 enum {
   SEESAW_TIMER_STATUS = 0x00,
   SEESAW_TIMER_PWM = 0x01,
   SEESAW_TIMER_FREQ = 0x02,
 };

 /** ADC module function address registers
  */
 enum {
   SEESAW_ADC_STATUS = 0x00,
   SEESAW_ADC_INTEN = 0x02,
   SEESAW_ADC_INTENCLR = 0x03,
   SEESAW_ADC_WINMODE = 0x04,
   SEESAW_ADC_WINTHRESH = 0x05,
   SEESAW_ADC_CHANNEL_OFFSET = 0x07,
 };

 /** Sercom module function address registers
  */
 enum {
   SEESAW_SERCOM_STATUS = 0x00,
   SEESAW_SERCOM_INTEN = 0x02,
   SEESAW_SERCOM_INTENCLR = 0x03,
   SEESAW_SERCOM_BAUD = 0x04,
   SEESAW_SERCOM_DATA = 0x05,
 };

 /** neopixel module function address registers
  */
 enum {
   SEESAW_NEOPIXEL_STATUS = 0x00,
   SEESAW_NEOPIXEL_PIN = 0x01,
   SEESAW_NEOPIXEL_SPEED = 0x02,
   SEESAW_NEOPIXEL_BUF_LENGTH = 0x03,
   SEESAW_NEOPIXEL_BUF = 0x04,
   SEESAW_NEOPIXEL_SHOW = 0x05,
 };


 /** touch module function address registers
  */
 enum {
   SEESAW_TOUCH_CHANNEL_OFFSET = 0x10,
 };

 /** keypad module function address registers
  */
 enum {
   SEESAW_KEYPAD_STATUS = 0x00,
   SEESAW_KEYPAD_EVENT = 0x01,
   SEESAW_KEYPAD_INTENSET = 0x02,
   SEESAW_KEYPAD_INTENCLR = 0x03,
   SEESAW_KEYPAD_COUNT = 0x04,
   SEESAW_KEYPAD_FIFO = 0x10,
 };

 /** keypad module edge definitions
  */
 enum {
   SEESAW_KEYPAD_EDGE_HIGH = 0,
   SEESAW_KEYPAD_EDGE_LOW,
   SEESAW_KEYPAD_EDGE_FALLING,
   SEESAW_KEYPAD_EDGE_RISING,
 };

 /** encoder module edge definitions
  */
 enum {
   SEESAW_ENCODER_STATUS = 0x00,
   SEESAW_ENCODER_INTENSET = 0x10,
   SEESAW_ENCODER_INTENCLR = 0x20,
   SEESAW_ENCODER_POSITION = 0x30,
   SEESAW_ENCODER_DELTA = 0x40,
 };


#define ADC_INPUT_0_PIN 2 ///< default ADC input pin
#define ADC_INPUT_1_PIN 3 ///< default ADC input pin
#define ADC_INPUT_2_PIN 4 ///< default ADC input pin
#define ADC_INPUT_3_PIN 5 ///< default ADC input pin

#define PWM_0_PIN 4 ///< default PWM output pin
#define PWM_1_PIN 5 ///< default PWM output pin
#define PWM_2_PIN 6 ///< default PWM output pin
#define PWM_3_PIN 7 ///< default PWM output pin

#define SEESAW_HW_ID_CODE_SAMD09 0x55 ///< seesaw HW ID code for SAMD09
#define SEESAW_HW_ID_CODE_TINY806 0x84 ///< seesaw HW ID code for ATtiny806
#define SEESAW_HW_ID_CODE_TINY807 0x85 ///< seesaw HW ID code for ATtiny807
#define SEESAW_HW_ID_CODE_TINY816 0x86 ///< seesaw HW ID code for ATtiny816
#define SEESAW_HW_ID_CODE_TINY817 0x87 ///< seesaw HW ID code for ATtiny817
#define SEESAW_HW_ID_CODE_TINY1616 0x88 ///< seesaw HW ID code for ATtiny1616
#define SEESAW_HW_ID_CODE_TINY1617 0x89 ///< seesaw HW ID code for ATtiny1617

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define INPUT_PULLDOWN 0x3

typedef struct {
 	I2C_HandleTypeDef *i2ch;
 	int i2caddr;
 	uint8_t hwtype;
} seesaw_t;

// i2caddr need only be the three address bits for the part (0 .. 7).
// returns 1 on success, 0 on failure
int seesaw_Begin(I2C_HandleTypeDef *hi2c,  int i2caddr, seesaw_t *ss);

int seesaw_SWReset(seesaw_t *ss);

uint32_t seesaw_getOptions(seesaw_t *ss);
uint32_t seesaw_getVersion(seesaw_t *ss);

int32_t seesaw_getEncoderPos(seesaw_t *ss, uint8_t encoder);
int32_t seesaw_getEncoderDelta(seesaw_t *ss, uint8_t encoder);
int seesaw_pinModeBulk(uint32_t pins, uint8_t mode, seesaw_t *ss);
int seesaw_pinMode(uint8_t pin, uint8_t mode, seesaw_t *ss);
int seesaw_enableEncoderInterrupt(seesaw_t *ss, uint8_t encoder);
int seesaw_setGPIOInterrupts(uint32_t pins, int enabled, seesaw_t *ss);
uint16_t seesaw_analogRead(uint8_t pin, seesaw_t *ss);
int seesaw_digitalRead(uint8_t pin, seesaw_t *ss);
uint32_t seesaw_digitalReadBulk(uint32_t pins, seesaw_t *ss);

#ifdef __cplusplus
}
#endif

#endif //ifndef __SEESAW_H
