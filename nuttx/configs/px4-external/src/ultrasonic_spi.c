/************************************************************************************
 * configs/stm32_tiny/src/stm32_wireless.c
 *
 *   Copyright (C) 2009, 2013 Gregory Nutt. All rights reserved.
 *   Author: Laurent Latil <laurent@latil.nom.fr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>


#include <nuttx/spi/spi.h>
#include <arch/board/board.h>




#include "up_arch.h"
#include "chip.h"
#include "stm32.h"
#include "px4_extern.h"
#include "stm32_capture.h"
#include  <nuttx/ly_ultrasonic/ultra_sonic_nuttx.h>


/* ultrasonic preprocess define */
#define GPIO_ULTRASONIC_TRIG    (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_10MHz|\
                                GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN11)
#define GPIO_BOARD_LED          (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_10MHz|\
                                GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN10)
#define GPIO_ULTRASONIC_IRQ     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTB|GPIO_PIN13)

#define SPI_CE_PIN (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_10MHz|\
    GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN14)

/* timer capture preprocess define */
#define TIM_CAP_CHN     (2)

/* private data */
struct spi_dev_s *uls_spi;
struct stm32_cap_dev_s *uls_cap;
static int stm32_uls_irq_attach(xcpt_t isr);
static void stm32_uls_trigonce(void);

ultrasonic_cfg_s cfg={

    stm32_uls_irq_attach,
    stm32_uls_trigonce,
    stm32_led_toggol
};

/* capture config */
static int stm32_uls_cap_config(xcpt_t isr)
{
    /* basic */
//    STM32_CAP_SETCLOCK(uls_cap, );
    /* config ccer ccmr */
//    STM32_CAP_SETCHANNEL(uls_cap, TIM_CAP_CHN, );/* TODO.. */

    /* ser isr callback function */
//    STM32_CAP_SETISR(uls_cap, );

    /* enable ch1 irq */
    STM32_CAP_ENABLEINT(uls_cap, STM32_CAP_FLAG_IRQ_CH_1, true);
}



/**
 * @brief sonar trig pin and irq
 * @param enable
 */
static int stm32_uls_irq_attach(xcpt_t isr)
{

    stm32_gpiosetevent(GPIO_ULTRASONIC_IRQ, false, true, false, isr);
    return OK;
}

static void stm32_uls_trigonce(void)
{

        stm32_gpiowrite(GPIO_ULTRASONIC_TRIG,1);
        usleep(10);
        stm32_gpiowrite(GPIO_ULTRASONIC_TRIG,0);
}

static void stm32_led_toggol(bool on)
{
    on?stm32_gpiowrite(GPIO_BOARD_LED,0):
       stm32_gpiowrite(GPIO_BOARD_LED,1);
}

/**
 * the spi cs and status need to offer by mcu layer
 */
void stm32_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
                      bool selected)
{
    selected?(stm32_gpiowrite(SPI_CE_PIN, false)):
             (stm32_gpiowrite(SPI_CE_PIN, true));
}

uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  return 0;
}


/**
 * @brief ultrasonic_spi_stm32_initialized
 * @return
 * intialized all of ultrasonic thing
 */
bool initialized = false;
int ultrasonic_spi_stm32_initialized(int argc, char *argv[])
{
    int err_code = OK;
    int ret;
    if(!initialized)
    {

        /*SPI INIT*/
        uls_spi = stm32_spibus_initialize(2);
        if(!uls_spi)
        {
            printf("SPI Initialized Error! %d\n",ret);
            goto fail;
        }

        /* GPIO INIT */
        stm32_configgpio(GPIO_BOARD_LED);
        stm32_configgpio(GPIO_ULTRASONIC_TRIG);
        stm32_configgpio(SPI_CE_PIN);
        ret = ultrasonic_register(uls_spi, &cfg);

        /* DEBUG INFO */
        if(ret < 0)
        {
            printf("registed error %d\n",ret);
            goto fail;
        }

        initialized = true;

    }
fail:
    printf("Try to initlized the sonar failed! \n");

    return err_code;
}
