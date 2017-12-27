/****************************************************************************
 * myapps/ultrasonic/ultra_sonic_nuttx.c
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
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <poll.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

//porting
#include <sys/time.h>
#include <semaphore.h>

#include <nuttx/ly_ultrasonic/ultra_sonic_nuttx.h>
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define ULTRAZ_SONIC_SPIFREQ   1000*10000

/****************************************************************************
 * Private Types
 ****************************************************************************/
struct ultrasonic_dev_s{

  FAR struct spi_dev_s *spi;   /* SPI bus*/
  FAR ultrasonic_cfg_s *config;    /*config callback fuctions*/

  sem_t sem_tx;
  bool ce_enabled;          /* Cache the value of CE pin */

  uint8_t nopens;           /* Number of times the device has been opened */
  sem_t devsem;             /* Ensures exclusive access to this structure */
  ulltrasonic_state_t state;

#ifndef CONFIG_DISABLE_POLL
  FAR struct pollfd *pfd;   /* Polled file descr  (or NULL if any) */
#endif

    struct timeval echo_begin;
    struct timeval echo_end;

};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/*Low-level SPI functions*/
static inline void ultrasonic_configspi(FAR struct spi_dev_s *spi);
static void ultrasonic_lock(FAR struct spi_dev_s *spi);
static void ultrasonic_unlock(FAR struct spi_dev_s *spi);

static int ultrasonic_irqhandler(FAR int irq, FAR void *context);

static inline int ultrasonic_attachirq(FAR struct ultrasonic_dev_s *dev, xcpt_t isr);

static inline bool ultrasonic_chipenable(FAR struct ultrasonic_dev_s *dev, bool enable);

/* POSIX API */

static int ultrasonic_open(FAR struct file *filep);

static int ultrasonic_close(FAR struct file *filep);

static ssize_t ultrasonic_read(FAR struct file *filep, FAR char *buffer, size_t buflen);

static ssize_t ultrasonic_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);

static int ultrasonic_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static int ultrasonic_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static FAR struct ultrasonic_dev_s *g_ultrasonic_dev;

static const struct file_operations ultrasonic_fops =
{
  ultrasonic_open,      /* open */
  ultrasonic_close,     /* close */
  ultrasonic_read,      /* read */
  ultrasonic_write,     /* write */
  NULL,                 /* seek */
  ultrasonic_ioctl,     /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  ultrasonic_poll,      /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  NULL                  /* unlink */
#endif
};
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/**
 * @brief ultrasonic_lock
 * @param spi
 */
static void ultrasonic_lock(FAR struct spi_dev_s *spi)
{
    /**
     * Lock the SPI bus because there are multiple devices competing for the
     * SPI bus
     */
    (void)SPI_LOCK(spi, true);

    /**
     *  We have the lock.  Now make sure that the SPI bus is configured for the
     *  ultrasonic (it might have gotten configured for a different device while
     *  unlocked)
     */

    SPI_SELECT(spi, SPIDEV_WIRELESS, true);
    SPI_SETMODE(spi, SPIDEV_MODE0);
    SPI_SETBITS(spi, 8);
    (void)SPI_HWFEATURES(spi, 0);
    (void)SPI_SETFREQUENCY(spi, ULTRAZ_SONIC_SPIFREQ);
    SPI_SELECT(spi, SPIDEV_WIRELESS, false);
}

/**
 * @brief ultrasonic_unlock
 * @param spi
 */
static void ultrasonic_unlock(FAR struct spi_dev_s *spi)
{
  /* Relinquish the SPI bus. */

  (void)SPI_LOCK(spi, false);
}

/**
 * @brief ultrasonic_configspi
 * @param spi
 */
static inline void ultrasonic_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the ultrasonic module. */

  SPI_SELECT(spi, SPIDEV_WIRELESS, true);  /* Useful ? */
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  (void)SPI_HWFEATURES(spi, 0);
  (void)SPI_SETFREQUENCY(spi, ULTRAZ_SONIC_SPIFREQ);
  SPI_SELECT(spi, SPIDEV_WIRELESS, false);
}

/**
 * @brief ultrasonic_select
 * @param dev
 */
static inline void ultrasonic_select(FAR struct ultrasonic_dev_s * dev)
{
  SPI_SELECT(dev->spi, SPIDEV_WIRELESS, true);
}

/**
 * @brief ultrasonic_deselect
 * @param dev
 */
static inline void ultrasonic_deselect(struct ultrasonic_dev_s * dev)
{
  SPI_SELECT(dev->spi, SPIDEV_WIRELESS, false);
}

/**
 * @brief ultrasonic_irqhandler
 * @param irq
 * @param context
 * @return
 */
bool led_state = false;
static int ultrasonic_irqhandler(int irq, FAR void *context)
{
  FAR struct ultrasonic_dev_s *dev = g_ultrasonic_dev;


//  _info("*IRQ*");
    gettimeofday(&dev->echo_end,NULL);


  led_state = !led_state;
  g_ultrasonic_dev->config->toggol(led_state);
  /* Otherwise we simply wake up the send function */
  sem_post(&dev->sem_tx);  /* Wake up the send function */


  return OK;
}



/**
 * @brief ultrasonic_trig
 */
static inline void ultrasonic_trig(struct ultrasonic_dev_s *dev)
{
    dev->config->trigonce();
}

/**
 * @brief ultrasonic_attachirq
 * @param dev
 * @param isr
 * @return
 */
static inline int ultrasonic_attachirq(FAR struct ultrasonic_dev_s *dev, xcpt_t isr)
{
  return dev->config->irqattach(isr);
}

/**
 * @brief ultrasonic_chipenable
 * @param dev
 * @param enable
 * @return
 */
/*
static inline bool ultrasonic_chipenable(FAR struct ultrasonic_dev_s *dev, bool enable)
{
  if (dev->ce_enabled != enable)
    {
      dev->config->chipenable(enable);
      dev->ce_enabled = enable;
      return !enable;
    }
  else
    {
      return enable;
    }
}
*/

/* POSIX API */

/**
 * @brief ultrasonic_open
 * @param filep
 * @return
 */
static int ultrasonic_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct ultrasonic_dev_s *dev;
  int result;

//  _info("Opening ultrasonic dev\n");
  DEBUGASSERT(filep);
  inode = filep->f_inode;
  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct ultrasonic_dev_s *)inode->i_private;
  /* Get exclusive access to the driver data structure */

  if (sem_wait(&dev->devsem) < 0)
  {
    /* This should only happen if the wait was canceled by an signal */

    DEBUGASSERT(errno == EINTR);
    printf("sem_wait error \n");
    return -EINTR;
  }

  /* Check if device is not already used */

  if (dev->nopens > 0)
  {
    result = -EBUSY;
    goto errout;
  }

//  result = ultrasonic_init(dev);
  result = errno;
  if (!result)
  {
    dev->nopens++;
  }
errout:
  sem_post(&dev->devsem);


  return result;
}

/**
 * @brief ultrasonic_close
 * @param filep
 * @return
 */
static int ultrasonic_close(FAR struct file *filep)
{
    FAR struct inode *inode;
    FAR struct ultrasonic_dev_s *dev;

    DEBUGASSERT(filep);
    inode = filep->f_inode;

    DEBUGASSERT(inode && inode->i_private);
    dev  = (FAR struct ultrasonic_dev_s *)inode->i_private;

    /* Get exclusive access to the driver data structure */

    if (sem_wait(&dev->devsem) < 0)
      {
        /* This should only happen if the wait was canceled by an signal */

        DEBUGASSERT(errno == EINTR);
        return -EINTR;
      }

//    nrf24l01_changestate(dev, ST_POWER_DOWN);
    dev->nopens--;

    sem_post(&dev->devsem);
    return OK;
}

/**
 * @brief ultrasonic_read
 * @param filep
 * @param buffer
 * @param buflen
 * @return
 */
static ssize_t ultrasonic_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
    FAR struct inode *inode;
    FAR struct ultrasonic_dev_s *dev;
    int ret = errno;
    //Fix it ! read() return -1

    printf("read from sonar!\n");


    return ret;
}

/**
 * @brief ultrasonic_write
 * @param filep
 * @param buffer
 * @param buflen
 * @return
 */
static ssize_t ultrasonic_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
    int ret = errno;
    FAR struct inode *inode;
    FAR struct ultrasonic_dev_s *dev;


    DEBUGASSERT(filep);
    inode = filep->f_inode;

    DEBUGASSERT(inode && inode->i_private);
    dev  = (FAR struct ultrasonic_dev_s *)inode->i_private;

    /* Get exclusive access to the driver data structure */

    if (sem_wait(&dev->devsem) < 0)
    {
    /* This should only happen if the wait was canceled by an signal */

    DEBUGASSERT(errno == EINTR);
    return -EINTR;
    }

    /* trig pin raise up an edge for at least 10us*/
    dev->config->trigonce();
    gettimeofday(&dev->echo_begin,NULL);

    sem_post(&dev->devsem);


    /* wait for irq */
    while(sem_wait(&dev->sem_tx) !=0)
    {

    }


    printf("%d\n",dev->echo_end.tv_usec- dev->echo_begin.tv_usec);

    return ret;
}

/**
 * @brief ultrasonic_ioctl
 * @param filep
 * @param cmd
 * @param arg
 * @return
 */
static int ultrasonic_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{

    switch(cmd)
    {
        case NONE:
            printf("Do Nothing! \n");
            break;
        default:break;

    }
    return errno;
}

/**
 * @brief ultrasonic_poll
 * @param filep
 * @param fds
 * @param setup
 * @return
 */
static int ultrasonic_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup)
{

    int ret = errno;
    return ret;
}


/****************************************************************************
 * Public Function
 ****************************************************************************/
/**
 * @brief ultrasonic_register
 * @param spi
 * @param cfg
 * @return
 */
int ultrasonic_register(FAR struct spi_dev_s *spi, FAR ultrasonic_cfg_s *cfg)
{

  FAR struct ultrasonic_dev_s *dev;
  int result = OK;

  ASSERT((spi != NULL) & (cfg != NULL));

  if ((dev = kmm_malloc(sizeof(struct ultrasonic_dev_s))) == NULL)
  {
    return -ENOMEM;
  }

  dev->spi = spi;
  dev->config = cfg;

  dev->state = UL_UNKNOWN;
  dev->ce_enabled = false;

  sem_init(&(dev->devsem), 0, 1);
  dev->nopens = 0;

#ifndef CONFIG_DISABLE_POLL
  dev->pfd = NULL;
#endif

  sem_init(&dev->sem_tx, 0, 0);
  sem_setprotocol(&dev->sem_tx, SEM_PRIO_NONE);

  /* Set the global reference */

  g_ultrasonic_dev = dev;

  /* Configure IRQ pin  (falling edge) */

  ultrasonic_attachirq(dev, ultrasonic_irqhandler);

  /* Register the device as an input device */

//  iinfo("Registering " ULTRASONIC_PATH "\n");

  result = register_driver(ULTRASONIC_PATH, &ultrasonic_fops, 0666, dev);
  if (result < 0)
  {
    //werr("ERROR: register_driver() failed: %d\n", result);
    //nrf24l01_unregister(dev);
      printf("r error!");
  }

  return result;
}
