/*
 ============================================================================
 Name        : register.c
 Author      : 
 Version     :
 Copyright   : Your copyright notice
 Description : calculate STM32 I2C timing
               Derived from Arduino STM32 twi.c

  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************

 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
uint32_t i2c_computeTiming(uint32_t clkSrcFreq, uint32_t i2c_speed);
#define I2C_VALID_TIMING_NBR          64U

/* Generic definition for series requiring I2C timing calculation */

#ifndef I2C_VALID_TIMING_NBR
#define I2C_VALID_TIMING_NBR          8U
#endif
#define I2C_ANALOG_FILTER_DELAY_MIN  50U /* ns */
#ifndef I2C_ANALOG_FILTER_DELAY_MAX
#define I2C_ANALOG_FILTER_DELAY_MAX 260U /* ns */
#endif
#ifndef I2C_USE_ANALOG_FILTER
#define I2C_USE_ANALOG_FILTER         1U
#endif
#ifndef I2C_DIGITAL_FILTER_COEF
#define I2C_DIGITAL_FILTER_COEF       0U
#endif
#define I2C_PRESC_MAX                16U
#define I2C_SCLDEL_MAX               16U
#define I2C_SDADEL_MAX               16U
#define I2C_SCLH_MAX                256U
#define I2C_SCLL_MAX                256U
#define SEC2NSEC            1000000000UL

typedef enum {
  I2C_SPEED_FREQ_STANDARD,  /* 100 kHz */
  I2C_SPEED_FREQ_FAST,      /* 400 kHz */
  I2C_SPEED_FREQ_FAST_PLUS, /* 1 MHz */
  I2C_SPEED_FREQ_NUMBER     /* Must be the last entry */
} I2C_speed_freq_t;

typedef struct {
  uint32_t input_clock;      /* I2C Input clock */
  uint32_t timing;           /* I2C timing corresponding to Input clock */
} I2C_timing_t;

static I2C_timing_t I2C_ClockTiming[I2C_SPEED_FREQ_NUMBER] = {0};

typedef struct {
  uint32_t freq;      /* Frequency in Hz */
  uint32_t freq_min;  /* Minimum frequency in Hz */
  uint32_t freq_max;  /* Maximum frequency in Hz */
  uint16_t hddat_min; /* Minimum data hold time in ns */
  uint16_t vddat_max; /* Maximum data valid time in ns */
  uint16_t sudat_min; /* Minimum data setup time in ns */
  uint16_t lscl_min;  /* Minimum low period of the SCL clock in ns */
  uint16_t hscl_min;  /* Minimum high period of SCL clock in ns */
  uint16_t trise;     /* Rise time in ns */
  uint16_t tfall;     /* Fall time in ns */
  uint8_t dnf;       /* Digital noise filter coefficient */
} I2C_Charac_t;

static const I2C_Charac_t I2C_Charac[] = {
  [I2C_SPEED_FREQ_STANDARD] =
  {
    .freq = 100000,
    .freq_min = 80000,
    .freq_max = 120000,
    .hddat_min = 0,
    .vddat_max = 3450,
    .sudat_min = 250,
    .lscl_min = 4700,
    .hscl_min = 4000,
    .trise = 640,
    .tfall = 20,
    .dnf = I2C_DIGITAL_FILTER_COEF,
  },
  [I2C_SPEED_FREQ_FAST] =
  {
    .freq = 400000,
    .freq_min = 320000,
    .freq_max = 480000,
    .hddat_min = 0,
    .vddat_max = 900,
    .sudat_min = 100,
    .lscl_min = 1300,
    .hscl_min = 600,
    .trise = 250,
    .tfall = 100,
    .dnf = I2C_DIGITAL_FILTER_COEF,
  },
  [I2C_SPEED_FREQ_FAST_PLUS] =
  {
    .freq = 1000000,
    .freq_min = 800000,
    .freq_max = 1200000,
    .hddat_min = 0,
    .vddat_max = 450,
    .sudat_min = 50,
    .lscl_min = 500,
    .hscl_min = 260,
    .trise = 60,
    .tfall = 100,
    .dnf = I2C_DIGITAL_FILTER_COEF,
  }
};

int main(void) {
	uint32_t ret = 0;
	puts("!!!Hello World!!!"); /* prints !!!Hello World!!! */
    ret = i2c_computeTiming(72000000, I2C_SPEED_FREQ_STANDARD);
    if (ret == 0xFFFFFFFF) {
    	printf("No valid configuration!\n");
    }
    else {
    	printf("I2C configuration = %i\n", ret);
    }
	return EXIT_SUCCESS;
}

/**
* @brief Calculate PRESC, SCLDEL, SDADEL, SCLL and SCLH and find best configuration.
* @param clkSrcFreq I2C source clock in HZ.
* @param i2c_speed I2C frequency (index).
* @retval config index (0 to I2C_VALID_TIMING_NBR], 0xFFFFFFFF for no
valid config.
*/
uint32_t i2c_computeTiming(uint32_t clkSrcFreq, uint32_t i2c_speed)
{
  uint32_t ret = 0xFFFFFFFFU;
  uint32_t valid_timing_nbr = 0;
  uint32_t ti2cclk;
  uint32_t ti2cspeed;
  uint32_t prev_error;
  uint32_t dnf_delay;
  uint32_t clk_min, clk_max;
  uint16_t scll, sclh;
  uint8_t prev_presc = I2C_PRESC_MAX;

  int32_t tsdadel_min, tsdadel_max;
  int32_t tscldel_min;
  uint8_t presc, scldel, sdadel;
  uint32_t tafdel_min, tafdel_max;

  if (i2c_speed < I2C_SPEED_FREQ_NUMBER) {

    /* Don't compute timing if already available value for the requested speed with the same I2C input frequency */
    if ((I2C_ClockTiming[i2c_speed].input_clock == clkSrcFreq) && (I2C_ClockTiming[i2c_speed].timing != 0U)) {
      ret = I2C_ClockTiming[i2c_speed].timing;
    } else {
      /* Save the I2C input clock for which the timing will be saved */
      I2C_ClockTiming[i2c_speed].input_clock = clkSrcFreq;

      ti2cclk = (SEC2NSEC + (clkSrcFreq / 2U)) / clkSrcFreq;
      ti2cspeed = (SEC2NSEC + (I2C_Charac[i2c_speed].freq / 2U)) / I2C_Charac[i2c_speed].freq;

      tafdel_min = (I2C_USE_ANALOG_FILTER == 1U) ? I2C_ANALOG_FILTER_DELAY_MIN : 0U;
      tafdel_max = (I2C_USE_ANALOG_FILTER == 1U) ? I2C_ANALOG_FILTER_DELAY_MAX : 0U;
      /*
       * tDNF = DNF x tI2CCLK
       * tPRESC = (PRESC+1) x tI2CCLK
       * SDADEL >= {tf +tHD;DAT(min) - tAF(min) - tDNF - [3 x tI2CCLK]} / {tPRESC}
       * SDADEL <= {tVD;DAT(max) - tr - tAF(max) - tDNF- [4 x tI2CCLK]} / {tPRESC}
       */
      tsdadel_min = (int32_t)I2C_Charac[i2c_speed].tfall +
                    (int32_t)I2C_Charac[i2c_speed].hddat_min -
                    (int32_t)tafdel_min - (int32_t)(((int32_t)I2C_Charac[i2c_speed].dnf +
                                                     3) * (int32_t)ti2cclk);
      tsdadel_max = (int32_t)I2C_Charac[i2c_speed].vddat_max -
                    (int32_t)I2C_Charac[i2c_speed].trise -
                    (int32_t)tafdel_max - (int32_t)(((int32_t)I2C_Charac[i2c_speed].dnf +
                                                     4) * (int32_t)ti2cclk);
      /* {[tr+ tSU;DAT(min)] / [tPRESC]} - 1 <= SCLDEL */
      tscldel_min = (int32_t)I2C_Charac[i2c_speed].trise +
                    (int32_t)I2C_Charac[i2c_speed].sudat_min;
      if (tsdadel_min <= 0) {
        tsdadel_min = 0;
      }
      if (tsdadel_max <= 0) {
        tsdadel_max = 0;
      }

      /* tDNF = DNF x tI2CCLK */
      dnf_delay = I2C_Charac[i2c_speed].dnf * ti2cclk;

      clk_max = SEC2NSEC / I2C_Charac[i2c_speed].freq_min;
      clk_min = SEC2NSEC / I2C_Charac[i2c_speed].freq_max;

      prev_error = ti2cspeed;

      for (presc = 0; presc < I2C_PRESC_MAX; presc++) {
        for (scldel = 0; scldel < I2C_SCLDEL_MAX; scldel++) {
          /* TSCLDEL = (SCLDEL+1) * (PRESC+1) * TI2CCLK */
          uint32_t tscldel = (scldel + 1U) * (presc + 1U) * ti2cclk;
          if (tscldel >= (uint32_t)tscldel_min) {

            for (sdadel = 0; sdadel < I2C_SDADEL_MAX; sdadel++) {
              /* TSDADEL = SDADEL * (PRESC+1) * TI2CCLK */
              uint32_t tsdadel = (sdadel * (presc + 1U)) * ti2cclk;
              if ((tsdadel >= (uint32_t)tsdadel_min) && (tsdadel <=
                                                         (uint32_t)tsdadel_max)) {
                if (presc != prev_presc) {
                  valid_timing_nbr ++;
                  if (valid_timing_nbr >= I2C_VALID_TIMING_NBR) {
                    return ret;
                  }
                  /* tPRESC = (PRESC+1) x tI2CCLK*/
                  uint32_t tpresc = (presc + 1U) * ti2cclk;
                  for (scll = 0; scll < I2C_SCLL_MAX; scll++) {
                    /* tLOW(min) <= tAF(min) + tDNF + 2 x tI2CCLK + [(SCLL+1) x tPRESC ] */
                    uint32_t tscl_l = tafdel_min + dnf_delay + (2U * ti2cclk) + ((scll + 1U) * tpresc);
                    /* The I2CCLK period tI2CCLK must respect the following conditions:
                    tI2CCLK < (tLOW - tfilters) / 4 and tI2CCLK < tHIGH */
                    if ((tscl_l > I2C_Charac[i2c_speed].lscl_min) &&
                        (ti2cclk < ((tscl_l - tafdel_min - dnf_delay) / 4U))) {
                      for (sclh = 0; sclh < I2C_SCLH_MAX; sclh++) {
                        /* tHIGH(min) <= tAF(min) + tDNF + 2 x tI2CCLK + [(SCLH+1) x tPRESC] */
                        uint32_t tscl_h = tafdel_min + dnf_delay + (2U * ti2cclk) + ((sclh + 1U) * tpresc);
                        /* tSCL = tf + tLOW + tr + tHIGH */
                        uint32_t tscl = tscl_l + tscl_h + I2C_Charac[i2c_speed].trise +
                                        I2C_Charac[i2c_speed].tfall;
                        if ((tscl >= clk_min) && (tscl <= clk_max) &&
                            (tscl_h >= I2C_Charac[i2c_speed].hscl_min) && (ti2cclk < tscl_h)) {
                          int32_t error = (int32_t)tscl - (int32_t)ti2cspeed;
                          if (error < 0) {
                            error = -error;
                          }
                          /* look for the timings with the lowest clock error */
                          if ((uint32_t)error < prev_error) {
                            prev_error = (uint32_t)error;
                            ret = ((presc & 0x0FU) << 28) | \
                                  ((scldel & 0x0FU) << 20) | \
                                  ((sdadel & 0x0FU) << 16) | \
                                  ((sclh & 0xFFU) << 8) | \
                                  ((scll & 0xFFU) << 0);
                            prev_presc = presc;
                            /* Save I2C Timing found for further reuse (and avoid to compute again) */
                            I2C_ClockTiming[i2c_speed].timing = ret;
                            //printf("tsdadel = %i\n", tsdadel);
                            //printf("tscldel = %i\n", tscldel);
                        	printf("I2C configuration = %i\n", ret);
                         }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  return ret;
}
