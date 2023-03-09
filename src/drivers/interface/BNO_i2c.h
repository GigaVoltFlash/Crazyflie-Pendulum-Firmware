
/*
 * Functinos for reading/writing i2c for BNO
 */
#ifndef _BNO_I2C_H
#define _BNO_I2C_H

#include "i2cdev.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "sh2_util.h"
#include "shtp.h"
#include "sh2_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include "string.h"
#include <math.h>

#define BNO08x_I2CADDR_DEFAULT 0x4A ///< The default I2C address
#define MAX_BUFFER_SIZE 32 // the standard buffer size

bool begin_I2C(uint8_t i2c_addr, int32_t sensor_id);
void hardwareReset(void);
bool wasReset(void);

bool enableReport(sh2_SensorId_t sensor, uint32_t interval_us);
bool getSensorEvent(sh2_SensorValue_t *value);


bool _init(int32_t sensor_id);


int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us);
void i2chal_close(sh2_Hal_t *self);
int i2chal_open(sh2_Hal_t *self);

uint32_t hal_getTimeUs(sh2_Hal_t *self);
void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent);


void quaternionToEuler(float qr, float qi, float qj, float qk, float *yaw, float *pitch, float *roll);

// Own implementation of min function
inline uint32_t int_min(uint32_t first_arg, uint32_t second_arg) {
    uint32_t return_val = 0;
    if (first_arg < second_arg) {
        return_val = first_arg;
    } else {
        return_val = second_arg;
    }
    return return_val;
}

#endif