/****************************************************************************
 * myapps/ultrasonic/ultra_sonic.c
 *
 *   Copyright (C) 2011-2012, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/wdog.h>

#include <nuttx/ly_ultrasonic/ultra_sonic_nuttx.h>

#define uls_sec_period  1000000/CONFIG_USEC_PER_TICK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/


/****************************************************************************
 * Private Data
 ****************************************************************************/
int counter = 0;
WDOG_ID wid;
int fd;
char buf[10] = "1234";

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
wdentry_t cycle(int arg);

/****************************************************************************
 * Name: ultra_sonic_main()
 ****************************************************************************/
int ultra_sonic_main(int argc, char *argv[])
{
    ultrasonic_spi_stm32_initialized();

     /* do opreation */
    fd = open("/dev/ultrasonic",O_WRONLY);
    if(fd < 0)
    {
        printf("/dev/ultrasonic open error\n");
        exit(1);
    }
    printf("/dev/ultrasonic successful start!\n");

    /* measure cycle */
    wid = wd_create();
    wd_start(wid, uls_sec_period, cycle, 0);

    //cmd = atoi(argv[2]);
    while(1)
    {
        write(fd,buf,1);
        printf("%d\n",counter);

        sleep(1);
    }


    //close(fd);
    return errno;
}

/**
 * @brief ent
 * @param arg
 * @return
 */
wdentry_t cycle(int arg)
{
    //read(fd,buf,1);
    counter ++;
    printf("cycle\n");
    wd_start(wid, uls_sec_period, cycle, 0);

    return 0;
}
