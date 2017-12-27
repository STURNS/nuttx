#ifndef ULTRA_SONIC_NUTTX_H
#define ULTRA_SONIC_NUTTX_H

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>

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
    void (*chipenable)(bool enable);
}ultrasonic_cfg_s ;


/* declare */
int ultrasonic_register(FAR struct spi_dev_s *spi, FAR ultrasonic_cfg_s *cfg);
#endif // ULTRA_SONIC_NUTTX_H

