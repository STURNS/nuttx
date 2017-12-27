#ifndef ULTRA_SONIC_NUTTX_H
#define ULTRA_SONIC_NUTTX_H

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>


#define ULTRASONIC_PATH   "/dev/ultrasonic"

extern bool led_state;

typedef enum
{
  UL_UNKNOWN,
} ulltrasonic_state_t;

typedef struct{

    /**
     * @ callbcak functions
     * @ initialized in ultra_sonnic_stm32.c
     */

    int  (*irqattach)(xcpt_t isr);
    void (*trigonce)(void);
    void (*toggol)(bool on);

}ultrasonic_cfg_s ;

enum{
    NONE = 0,
    IOCTL_MEASURE_ONCE,
    IOCTL_MEASURE_CYCLE,
    IOCTL_MEASURE_STOP,
    IOCTL_TRANSIMIT

}IOCTL_NO;

/* declare */

/**
 * @brief ultrasonic_register
 * @param spi
 * @param cfg
 * @return
 */
int ultrasonic_register(FAR struct spi_dev_s *spi, FAR ultrasonic_cfg_s *cfg);

/**
 * @brief ultrasonic_spi_stm32_initialized
 * @param argc
 * @param argv
 * @return
 */
int ultrasonic_spi_stm32_initialized();


#endif // ULTRA_SONIC_NUTTX_H

