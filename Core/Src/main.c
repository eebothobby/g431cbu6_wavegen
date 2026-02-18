/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  *
  * Copyright 2026 Ashok Singhal
  *
  * Create a DDS (Direct Digital Synthesis) system to generate waves on the 2
  * channels of a high speed DAC (DAC3) of an STM32G431CBU6 using circular DMA
  * from waveform data in memory driven by timers (6 and 7).
  * DAC3 is not connected directly to an output pin so both channels are
  * connected to OPAMPs in follower mode.
  * DAC3.channel1 -> OPAMP1 -> output pin PA2
  * DAC3.channel2 -> OPAMP3 -> output pin PB1
  *
  * Also create a programmable pulse generator using PWM using  timer TIM3
  * TIM3.pwm -> output pin PA6
  *
  * Use a ST7789-based LCD display on SPI1 to display the control information.
  * I adapt driver for this from https://github.com/Floyd-Fish/ST7789-STM32/tree/master/ST7789
  *
  * Use a seesaw device (derived from Adafruit's design) with 2 rotary encoders
  * with built-in switches and 4 pushbutton switches on I2C1 for the control input.
  *
  * The amplitudes of wave0 and wave1 are determined by MCP54HVX1 digital potentiometer
  * feedback resistor for the output opamp.
  * The offsets of wave0 and wave1 are determined by lower speed DAC1 outputs on
  * channels 1 and 2.
  *
  * The user interface:
  * ENC2: adjust frequency/wave type/amplitude/offset for the current wave
  * ENCSW2: select mode for the enc2
  * SW4: Set the changes from ENC1/ENCSW1
  * SW3: Select the current wave (curwave)
  *
  * ENC1: adjust frequency and duty cycle for PWM
  * SW2: Set PWM changes XXX Currently unused
  * SW1: (currently unused)
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "gwave.h"
#include "st7789.h"
#include "seesaw.h"
#include "mcp45hvx1.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SS_ENCSW1 13
#define SS_ENCSW2 16
#define SS_SW1 5
#define SS_SW2 4
#define SS_SW3 3
#define SS_SW4 0

#define MIN_PERIOD 5
#define MAX_PERIOD 65535
#define MAX_SAMPLES 500
#define MIN_SAMPLES 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac3;
DMA_HandleTypeDef hdma_dac3_ch1;
DMA_HandleTypeDef hdma_dac3_ch2;

I2C_HandleTypeDef hi2c1;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp3;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

// DAC channels, used for DAC3 (wave output) as well as DAC1 (offsets)
uint32_t dac_chan[2] = {DAC_CHANNEL_1, DAC_CHANNEL_2};

// Timer 3, 6, 7 timing parameters
TIM_HandleTypeDef *htim[2] = {&htim6, &htim7}; // timers for the two channels

uint16_t prescaler3 = 1-1; // 150 Mhz from 150 Mhz clock
uint16_t period3 = 150-1; // 1 MHz from 150 MHz
uint16_t pulse3 = 75-1; // 50% duty cycle (75/150)

// prescaler6, prescaler7 are set to 1 (2-1) to give 75MHz from 150MHz
uint16_t prescaler6 = 2-1;
uint16_t prescaler7 = 2-1;

// If MIN_PERIOD is 4 (5-1) maximum sample clock is 75MHz/5 = 15MHz
// I have found that we get a lot of jitter at 15MHz sample clock, even though the
// spec says that 15MHz should be possible, so instead I use MIN_PERIOD of 5 (6-1)
// so the fastest sample clock is 75MHz/6 = 12.5MHz

// Set initial values of the period
uint16_t period6 = 14; // (15 - 1) 5 MHz from 75 MHz (10KHz with 500 samples)
uint16_t period7 = 74; // (75 - 1) 1 MHz from 75 MHz (2Khz with 500 samples)

seesaw_t ss;
// list of seesaw switch pins
uint8_t sswpins[6] = {SS_SW1, SS_SW2, SS_SW3, SS_SW4, SS_ENCSW1, SS_ENCSW2};
// previous values for seesaw switches
int sswprevs[6];

mcp45hvx1_t dpots[2];


// encsw2Mode determines what the encoder changes (f = freq, a = amplitude, t = offset)
// 0: shape
// 1: freq
// 2: ampl
// 3: offs

unsigned int encsw2Mode = 0;

// encsw1Mode determines what the encoder changes
// 0: pulse period (freq)
// 1: pulse width (duty)

unsigned int encsw1Mode = 0;

uint16_t wave[2][MAX_SAMPLES];
uint16_t woffset[2] = {2047, 2047};
uint16_t wampl[2] = {127, 127};
float wampV[2]; // amplitude in volts
float woffV[2]; // offset in volts

int num_samples[2] = {MAX_SAMPLES, MAX_SAMPLES};
// actual number of samples can be smaller than MAX_SAMPLES
// When num_sample[i] is less than 1/2 MAX_SAMPLES, we duplicate
// the wave num_waves[i] times so that each DMA cycle can run for longer
// and we get fewer interrupts.
// num_waves[i] * num_samples[i] must be <= MAX_SAMPLES
int num_waves[2] = {1, 1};
// wtype0,1 sets the type of waveform in wave[0], wave[1]
// 0 = sin, 1 = rampup, 2 = rampdown, 3 = triangle, 4 = sinc
// sw2 cycles through these waveform types
char wnames[5][6] = {
		"sine", "rampu", "rampd", "trian", "sinc"
};
uint8_t wtype[2] = {0, 3};
uint8_t curwave = 0; // 0, 1 = wave0, wave1
uint32_t prescalerHz[2];  // frequency after prescaler
uint32_t sampleHz[2]; // sampling Frequency (after period)
float waveHz[2];
// Whether the waveHz or wtype has been changed and needs to be set
int waveChanged[2] = {0, 0};
float pwmhz; // pwm hz
float pwmdpct; // and duty cycle %

// USB virtual com port receive buffer
uint8_t usb_rec_buf[64];
uint8_t usb_rec_len;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_DAC3_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void USB_Printf(const char* fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    CDC_Transmit_FS((uint8_t *) buf, strlen(buf));
    va_end(args);
}

// Return the amplitude in volts
// Rpot = 10K * (255 - potv) / 255
// Gain = Rpot / 1.5K
//      = 10 * (255 - potv) / (255 * 1.5)
//      = 0.026144 * (255 - potv)
// Amplitude = Gain * 3.175V  = 0.085 * (255 - potv)
// And we add 0.3 because of minimum Rpot determined empirically

float amplitude(uint16_t potv) {
	return  0.3 + 0.083 * (255 - potv);
}

// Return the offset in volts
// Gain = 0.026144 * (255 - potv)
// Offset = 3.2 * dacv/4095 * Gain = 3.2 * 0.026144 * (255 -potv) * dacv/4095
//  = 0.083 * (255 -potv) * davc/4095
float offset(uint16_t dacv, uint16_t potv) {
	return 0.083 * (255 - potv) * dacv/4095.0;
}

// Display is 240x280 means a line can be 21 chars wide
// 0:  Wavegen
// 1: wave0 	wave1
// 2: freq0  	freq1 	set>
// 3: shape0  	shape1
// 4: ampl0	ampl1  	   enc1b>
// 5: off0		off1
// 6:				   waven>
// 7:  Pulsegen
// 8: Pp	period3    set>
// 9: pwmhz		duty  enc2b>
//
void Display() {
	char buf[32];
	// choose background color based on curwave and modes
	uint16_t fcol = WHITE, fcol_changed = YELLOW, bcol = BLACK, bcol_cur = GRAY;
	uint8_t row = 0, ysp = 20, xsp = 12;

	// row 0 Needs to start at X offset due to rounded edge
	ST7789_WriteString(0, row*ysp, "       Wavegen       ", Font_11x18, YELLOW, BLUE);

	// row 1
	row++;
	ST7789_WriteString(2*xsp, row * ysp, " Wave0", Font_11x18, waveChanged[0]? fcol_changed: fcol,
			(curwave == 0)? bcol_cur: bcol);
	ST7789_WriteString(8*xsp, row * ysp, " Wave1", Font_11x18, waveChanged[1]? fcol_changed: fcol,
			(curwave == 1)? bcol_cur: bcol);

	// row 2: Wave frequencies and Set> button (S4).  Needs to start at X offset due to rounded edge
	row++;
	ST7789_WriteString(0, row * ysp, "Hz", Font_11x18, YELLOW, BLUE);
	sprintf(buf, "%7d", (int) (waveHz[0]));
	ST7789_WriteString(2*xsp, row * ysp, buf, Font_11x18, waveChanged[0]? fcol_changed: fcol,
			(encsw2Mode > 0 && encsw2Mode < 6 && curwave == 0)? bcol_cur: bcol);
	sprintf(buf, "%7d", (int) (waveHz[1]));
	ST7789_WriteString(9*xsp, row * ysp, buf, Font_11x18, waveChanged[1]? fcol_changed: fcol,
			(encsw2Mode > 0 && encsw2Mode < 6 && curwave == 1)? bcol_cur: bcol);
	ST7789_WriteString(16*xsp, row * ysp, " Set", Font_11x18, BLUE, WHITE);

	// row 3: Wave types and current encoder 1 mode
	row++;
	ST7789_WriteString(0, row * ysp, "Sh", Font_11x18, YELLOW, BLUE);
	sprintf(buf, "%5s", wnames[wtype[0]]);
	ST7789_WriteString(2*xsp, row * ysp, buf, Font_11x18, fcol,
			(encsw2Mode == 0 && curwave == 0)? bcol_cur: bcol);
	sprintf(buf, "%5s", wnames[wtype[1]]);
	ST7789_WriteString(9*xsp, row * ysp, buf, Font_11x18, fcol,
			(encsw2Mode == 0 && curwave == 1)? bcol_cur: bcol);

	// row 4: Wave amplitudes and wave select button (S3)
	row++;
	ST7789_WriteString(0, row * ysp, "Am", Font_11x18, YELLOW, BLUE);
	sprintf(buf, "%7.3f", wampV[0]);
	ST7789_WriteString(2*xsp, row * ysp, buf, Font_11x18, fcol,
			(encsw2Mode == 2 && curwave == 0)? bcol_cur: bcol);
	sprintf(buf, "%7.3f", wampV[1]);
	ST7789_WriteString(9*xsp, row * ysp, buf, Font_11x18, fcol,
			(encsw2Mode == 2 && curwave == 1)? bcol_cur: bcol);
	switch (encsw2Mode) {
	case 0:
		sprintf(buf, "shape");
		break;
	case 1:
		sprintf(buf, " freq");
		break;
	case 2:
		sprintf(buf, " ampl");
		break;
	case 3:
		sprintf(buf, " offs");
		break;
	default:
		sprintf(buf, "????");
		break;
	}
	ST7789_WriteString(15*xsp, row * ysp, buf, Font_11x18, BLUE, WHITE);


	// row 5: Wave offsets
	row++;
	ST7789_WriteString(0, row * ysp, "Of", Font_11x18, YELLOW, BLUE);
	sprintf(buf, "%7.3f", woffV[0]);
	ST7789_WriteString(2*xsp, row * ysp, buf, Font_11x18, fcol,
			(encsw2Mode == 3 && curwave == 0)? bcol_cur: bcol);
	sprintf(buf, "%7.3f", woffV[1]);
	ST7789_WriteString(9*xsp, row * ysp, buf, Font_11x18, fcol,
			(encsw2Mode == 3 && curwave == 1)? bcol_cur: bcol);

	// Row6
	row++;
	ST7789_WriteString(16*xsp, row * ysp, curwave ? "Wav1" : "Wav0", Font_11x18, BLUE, WHITE);

	// row7 pulse header
	row++;
	ST7789_WriteString(0, row*ysp, "       Pulsegen      ", Font_11x18, YELLOW, BLUE);

	// row 8: Pp period Set>
	row++;
	ST7789_WriteString(0, row * ysp, "Pp", Font_11x18, YELLOW, BLUE);
	sprintf(buf, "%6d", period3);
	ST7789_WriteString(2*xsp, row * ysp, buf, Font_11x18, fcol,
			(encsw1Mode == 0)? bcol_cur: bcol);
	// XXX Set unused
	ST7789_WriteString(16*xsp, row * ysp, "    ", Font_11x18, BLUE, WHITE);

	// row 9: PWM freq
	row++;
	ST7789_WriteString(0, row * ysp, "Hz", Font_11x18, YELLOW, BLUE);
	sprintf(buf, "%7.0f", pwmhz);
	ST7789_WriteString(2*xsp, row * ysp, buf, Font_11x18, fcol, bcol);

	// row 10:Pw width Encb1>
	row++;
	ST7789_WriteString(0, row * ysp, "Pw", Font_11x18, YELLOW, BLUE);
	sprintf(buf, "%6d", pulse3);
	ST7789_WriteString(2*xsp, row * ysp, buf, Font_11x18, fcol,
			(encsw1Mode == 1)? bcol_cur: bcol);
	switch (encsw1Mode) {
	case 0:
		sprintf(buf, "pper");
		break;
	case 1:
		sprintf(buf, "pwid");
		break;
	default:
		sprintf(buf, "????");
		break;
	}
	ST7789_WriteString(15*xsp, row * ysp, buf, Font_11x18, BLUE, WHITE);

	// row 11: d% duty
	row++;
	ST7789_WriteString(0, row * ysp, "D%%", Font_11x18, YELLOW, BLUE);
	sprintf(buf, "%6.1f", pwmdpct);
	ST7789_WriteString(2*xsp, row * ysp, buf, Font_11x18, fcol, bcol);

	// row 12
	row++;
	// XXX Set unused
	ST7789_WriteString(16*xsp, row * ysp, "    ", Font_11x18, BLUE, WHITE);
}

void updateClocks() {
	sampleHz[0] = prescalerHz[0] / (period6 + 1);
	sampleHz[1] = prescalerHz[1] / (period7 + 1);
	USB_Printf("Sample clocks: %u %u\r\n", sampleHz[0], sampleHz[1]);
	waveHz[0] = sampleHz[0] / num_samples[0];
	waveHz[1] = sampleHz[1] / num_samples[1];
	USB_Printf("Wave clocks: %.1f %.1f\r\n", waveHz[0], waveHz[1]);
}


// calculate pwmhz (freq) and pwmdpct (duty cycle %) from TIM3 period3 and pulse3
void updatePwm() {
	  pwmhz = SystemCoreClock/(period3 + 1); // since the prescaler is set to 1
	  pwmdpct = (pulse3 + 1) * 100.0/(period3 + 1);
	  if (pwmdpct > 100) {
		  pwmdpct = 100;
	  }
}

// Set the frequency of curwave to freq
// Should only be called for curwave = 0 or 1
int setFreq(float freq) {
	float targetSampleFreq;
	uint16_t targetPeriod;
	int targetNumSamples;

	// max sample freq given the prescalerHz determined by MIN_PERIOD
	// and min sample freq is determined by MAX_PERIOD
	uint32_t maxSampleFreq = prescalerHz[curwave]/(MIN_PERIOD + 1);
	uint32_t minSampleFreq = prescalerHz[curwave]/(MAX_PERIOD + 1);
	// Calculate the min and max wave frequencies
	float minWaveFreq = minSampleFreq / MAX_SAMPLES;
	uint32_t maxWaveFreq = maxSampleFreq/ MIN_SAMPLES;
	// max freq with MAX_SAMPLES
	uint32_t maxMaxSampleFreq = maxSampleFreq/MAX_SAMPLES;

	if ((freq < minWaveFreq) || (freq > maxWaveFreq)) {
		USB_Printf("freq out of range\r\n");
		return 0;
	}

	if (freq <= maxMaxSampleFreq) {
		// Can use max samples
		targetSampleFreq = freq * MAX_SAMPLES;
		targetPeriod = (uint16_t) ceil((prescalerHz[curwave] / targetSampleFreq) - 1);
		if (targetPeriod < MIN_PERIOD) {
			targetPeriod = MIN_PERIOD;
		}
		targetSampleFreq = prescalerHz[curwave]/(targetPeriod + 1);
		// adjust num_samples to get closer to desired freq
		targetNumSamples = (int) (targetSampleFreq/freq);
		if (targetNumSamples > MAX_SAMPLES) {
			targetNumSamples = MAX_SAMPLES;
		}
	} else {
		// cannot use MAX_SAMPLES
		targetNumSamples = (int) (maxSampleFreq/freq);
		if (targetNumSamples < MIN_SAMPLES) {
			targetNumSamples = MIN_SAMPLES;
		}
		targetSampleFreq = targetNumSamples * freq;
		targetPeriod = (uint16_t) ((prescalerHz[curwave] / targetSampleFreq) - 1);
		if (targetPeriod < MIN_PERIOD) {
			targetPeriod = MIN_PERIOD;
		}
	}

	num_samples[curwave] = targetNumSamples;
	if (num_samples[curwave] < MAX_SAMPLES/2) {
		num_waves[curwave] = MAX_SAMPLES/num_samples[curwave];
	} else {
		num_waves[curwave] = 1;
	}
	initWave(wtype[curwave], wave[curwave], num_samples[curwave], num_waves[curwave]);
	if (HAL_DAC_Stop_DMA(&hdac3, dac_chan[curwave]) != HAL_OK) {
		USB_Printf("HAL_DAC_Stop_DMA failed");
	}
	if (HAL_DAC_Start_DMA(&hdac3, dac_chan[curwave], (uint32_t *) wave[curwave],
			num_samples[curwave] * num_waves[curwave], DAC_ALIGN_12B_R) != HAL_OK) {
		USB_Printf("HAL_DAC_Start_DMA failed");
	}
	if (curwave == 1) {
		period7 = targetPeriod;
	} else {
		period6 = targetPeriod;
	}
	__HAL_TIM_SET_AUTORELOAD(htim[curwave],targetPeriod);
	updateClocks();
	return 1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	int32_t encdiff0, encdiff1;
	int redisplay = 0;
	int sswCur;
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
  MX_DMA_Init();
  MX_USB_Device_Init();
  MX_I2C1_Init();
  MX_DAC3_Init();
  MX_OPAMP1_Init();
  MX_OPAMP3_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  USB_Printf("System Clock: %d\r\n", SystemCoreClock);
  prescalerHz[0] = SystemCoreClock / (prescaler6 + 1);
  prescalerHz[1] = SystemCoreClock / (prescaler7 + 1);
  USB_Printf("Prescaler clocks: %.1f %.1f\r\n", prescalerHz[0], prescalerHz[1]);

  updateClocks();
  updatePwm();

  ST7789_Init();
  ST7789_InvertColors(1);
  HAL_Delay(1);
  Display();

  USB_Printf("Scanning I2C\r\n");
  for (int i2a = 1; i2a < 127; i2a++) {
  	if (HAL_I2C_IsDeviceReady(&hi2c1, i2a << 1, 1, 10) == HAL_OK) {
  		USB_Printf("I2C device at %x\r\n", i2a);
  	}
  }
  USB_Printf("Scan I2C done\r\n");
  USB_Printf("Initializing seesaw\r\n");
  if (!seesaw_Begin(&hi2c1, 0, &ss)) {
  	USB_Printf("seesaw not found\r\n");
  }
  USB_Printf("seesaw_hw: %x\r\n", ss.hwtype);

  if (!seesaw_SWReset(&ss)) {
	  USB_Printf("SWReset failed\r\n");
  }

  HAL_Delay(1);

  // define pin modes for switch pins
  for (int swp = 0; swp < 6; swp++) {
	  if (!seesaw_pinMode(sswpins[swp], INPUT_PULLUP, &ss)) {
		  USB_Printf("seesaw_pinMode failed for %d\r\n", sswpins[swp]);
	  }
  }

  USB_Printf("Initializing dpots\r\n");
  for (int dpot = 0; dpot < 2; dpot++) {
	  if (!mcp45hvx1_Begin(&hi2c1, dpot, &dpots[dpot])) {
	 	  USB_Printf("mcp45hvx1_Begin() failed for dpot %d\r\n", dpot);
	  }
	  // Write initial value to dpot
	  mcp45hvx1_setWiper(wampl[dpot], &dpots[dpot]);
	  wampV[dpot] = amplitude(wampl[dpot]);
  }
  USB_Printf("Done initializing dpots\r\n");

  // initialize the two wave channels
  for (int wv = 0; wv < 2; wv++) {
	  initWave(wtype[wv], wave[wv], num_samples[wv], num_waves[wv]);
	  HAL_DAC_Start_DMA(&hdac3, dac_chan[wv], (uint32_t *) wave[wv], num_samples[wv], DAC_ALIGN_12B_R);
	  HAL_TIM_Base_Start(htim[wv]);
	  HAL_DAC_Start(&hdac1, dac_chan[wv]);
	  HAL_DAC_SetValue(&hdac1, dac_chan[wv], DAC_ALIGN_12B_R, woffset[wv]);
	  woffV[wv] = offset(woffset[wv], wampl[wv]);
  }
  //Avoid DMA half transfer, transfer complete interrupt trigger
  __HAL_DMA_DISABLE_IT(&hdma_dac3_ch1, DMA_IT_HT | DMA_IT_TC);
  __HAL_DMA_DISABLE_IT(&hdma_dac3_ch2, DMA_IT_HT | DMA_IT_TC);

  if (HAL_OPAMP_Start(&hopamp1)) {
	  USB_Printf("HAL_OPAMP_Start failed\r\n");
  }
  if (HAL_OPAMP_Start(&hopamp3)) {
  	  USB_Printf("HAL_OPAMP_Start failed\r\n");
  }
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  Display();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    redisplay = 0;
	  // process the 6 seesaw switches
	  for (uint8_t swp = 0; swp < 6; swp++) {
		  sswCur = seesaw_digitalRead(sswpins[swp], &ss);
		  if (sswCur != sswprevs[swp]) {
			  if (!sswCur) {
				  switch (sswpins[swp]) {
				  case SS_ENCSW2:
					  encsw2Mode = (encsw2Mode + 1) % 4;
					  redisplay = 1;
					  break;
				  case SS_SW4:
					  if (waveChanged[curwave]) {
						  if (setFreq(waveHz[curwave])) {
							  waveChanged[curwave] = 0;
						  }
					  }
					  redisplay = 1;
					  break;
				  case SS_SW3:
					  curwave = 1 - curwave;
					  redisplay = 1;
					  break;
				  case SS_SW2:
					  // XXX Unused
					  break;
				  case SS_ENCSW1:
					  encsw1Mode = (encsw1Mode + 1) % 2;
					  redisplay = 1;
					  break;
				  case SS_SW1:
					  // XXX not used
					  break;
				  }
			  }
			  sswprevs[swp] = sswCur;
		  }
	  }

	  // encoder1 adjusts wavetype/frequency/amplitude/offset for curwave
	  encdiff1 = seesaw_getEncoderDelta(&ss, 1);
	  if (encdiff1 != 0) {
		  USB_Printf("encdiff1: %d\r\n", encdiff1);
		  switch (encsw2Mode) {
		  case 0:
			  wtype[curwave] = (wtype[curwave] + encdiff1) % 5;
			  if (wtype[curwave] >= 5) wtype[curwave] = 4; // Seems to be a bug when encdiff1 is negative
			  initWave(wtype[curwave], wave[curwave], num_samples[curwave], num_waves[curwave]);
			  break;
		  case 1:
			  if (abs(encdiff1) == 2) {
				  encdiff1 *= 4;
			  } else if (abs(encdiff1) == 3) {
				  encdiff1 *= 16;
			  } else if (abs(encdiff1) == 4) {
				  encdiff1 *= 64;
			  } else if (abs(encdiff1) == 5) {
				  encdiff1 *= 256;
			  } else if (abs(encdiff1) == 6) {
				  encdiff1 *= 1024;
			  } else if (abs(encdiff1) > 6) {
				  encdiff1 *= 4096;
			  }
			  waveHz[curwave] = waveHz[curwave] + encdiff1;
			  waveChanged[curwave] = 1;
			  break;
		  case 2:
			  // Want the resistance (gain) to increase so we subtract
			  // instead of adding
			  if ((wampl[curwave] - encdiff1) > 255) {
				  wampl[curwave] = 255;
			  } else if (encdiff1 > wampl[curwave]) {
				  wampl[curwave] = 0;
			  } else {
				  wampl[curwave] = wampl[curwave] - encdiff1;
			  }
			  mcp45hvx1_setWiper(wampl[curwave], &dpots[curwave]);
			  wampV[curwave] = amplitude(wampl[curwave]);
			  woffV[curwave] = offset(woffset[curwave], wampl[curwave]);
			  break;
		  case 3:
			  if (abs(encdiff1) == 2) {
				  encdiff1 *= 10;
			  } else if (abs(encdiff1) > 2) {
				  encdiff1 *= 100;
			  }
			  if (encdiff1 > 0) {
				  woffset[curwave] = woffset[curwave] + encdiff1;
				  if (woffset[curwave] >= 4096) woffset[curwave] = 4095;
			  } else {
				  if (abs(encdiff1) > woffset[curwave]) {
					  woffset[curwave] = 0;
				  } else {
					  woffset[curwave] = woffset[curwave] + encdiff1;
				  }
			  }


			  HAL_DAC_SetValue(&hdac1, dac_chan[curwave], DAC_ALIGN_12B_R, woffset[curwave]);
			  woffV[curwave] = offset(woffset[curwave], wampl[curwave]);
			  break;
		  }
		  redisplay = 1;
	  }

	  // encoder0 adjusts frequency/duty for pulse gen
	  encdiff0 = seesaw_getEncoderDelta(&ss, 0);
	  if (encdiff0 != 0) {
		  USB_Printf("encdiff0: %d\r\n", encdiff0);
		  if (abs(encdiff0) == 2) {
			  encdiff0 *= 4;
		  } else if (abs(encdiff0) == 3) {
			  encdiff0 *= 16;
		  } else if (abs(encdiff0) > 3) {
			  encdiff0 *= 64;
		  }
		  switch (encsw1Mode) {
		  case 0:
			  // change period3
			  if (encdiff0 > 0) {
				  if (MAX_PERIOD - period3 <= encdiff0) {
					  period3 = MAX_PERIOD;
				  } else {
					  period3 += encdiff0;
				  }
			  } else {
				  // encdiff0 is -ve
				  if (period3 - MIN_PERIOD <= -encdiff0) {
					  period3 = MIN_PERIOD;
				  } else {
					  period3 += encdiff0;
				  }
			  }
			  __HAL_TIM_SET_AUTORELOAD(&htim3, period3);
			  if (pulse3 >= period3) {
				  pulse3 = period3 - 1;
				  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse3);
			  }
			  break;
		  case 1:
			  // change pulse3
			  if (encdiff0 > 0) {
				  if (period3 - pulse3 <= encdiff0) {
					  pulse3 = period3 - 1;
				  } else {
					  pulse3 += encdiff0;
				  }
			  } else {
				  // encdiff1 -ve
				  if (-encdiff0 >= pulse3) {
					  pulse3 = 1;
				  } else {
					  pulse3 += encdiff0;
				  }
			  }
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse3);
			  break;
		  }
		  updatePwm();
		  redisplay = 1;
	  }
	  if (redisplay) {
		  Display();
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DAC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC3_Init(void)
{

  /* USER CODE BEGIN DAC3_Init 0 */

  /* USER CODE END DAC3_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC3_Init 1 */

  /* USER CODE END DAC3_Init 1 */

  /** DAC Initialization
  */
  hdac3.Instance = DAC3;
  if (HAL_DAC_Init(&hdac3) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC3_Init 2 */

  /* USER CODE END DAC3_Init 2 */

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
  hi2c1.Init.Timing = 0x10B30E23;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
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

  /** I2C Fast mode Plus enable
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
  hopamp1.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_DAC;
  hopamp1.Init.InternalOutput = DISABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief OPAMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP3_Init(void)
{

  /* USER CODE BEGIN OPAMP3_Init 0 */

  /* USER CODE END OPAMP3_Init 0 */

  /* USER CODE BEGIN OPAMP3_Init 1 */

  /* USER CODE END OPAMP3_Init 1 */
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
  hopamp3.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_DAC;
  hopamp3.Init.InternalOutput = DISABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP3_Init 2 */

  /* USER CODE END OPAMP3_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = prescaler3;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = period3;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse3;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = prescaler6;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = period6;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = prescaler7;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = period7;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_DC_Pin|LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_DC_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LCD_RST_GPIO_Port, &GPIO_InitStruct);

  /**/
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(SYSCFG_FASTMODEPLUS_PB9);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
