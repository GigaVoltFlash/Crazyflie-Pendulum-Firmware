#define DEBUG_MODULE "bno08xDeck"
// All includes
#include "debug.h"
#include "i2cdev.h"
#include "deck.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include <math.h>
#include "BNO_i2c.h"
#include "stabilizer_types.h"

// Add constants for BNO here
float l = 62.0; // cm
#define BNO08x_TASK_NAME "BNO08x"
#define BNO08x_TASK_STACKSIZE (2 * configMINIMAL_STACK_SIZE)
#define BNO08x_TASK_PRI 3
#define BNO08x_I2CADDR_DEFAULT 0x4A ///< The default I2C address


// Misc global variables
static bool isInit;
float r_pos = 10.0;
float s_pos = 0;
float yaw;
float pitch;
float roll;
static bool I2C_began;
static bool reports_enabled;
uint8_t acquired_data[19];
static uint8_t got_data;
int status_log;

void bno08xTask(void* arg);

// Deck driver init function
static void bno08xInit()
{
  if (isInit)
    return;

  DEBUG_PRINT("Initializing BNO08x...\n");
  i2cdevInit(I2C1_DEV);

  I2C_began = begin_I2C((uint8_t)BNO08x_I2CADDR_DEFAULT, 0);
  r_pos = 15.0;
  if (!I2C_began) {
    return;
  }

  reports_enabled = enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  if (!reports_enabled) {
    return;
  }

  xTaskCreate(bno08xTask, BNO08x_TASK_NAME, BNO08x_TASK_STACKSIZE, NULL, BNO08x_TASK_PRI, NULL);

  isInit = true;
  DEBUG_PRINT("BNO08x initialization complete!\n");
}

// Deck driver test function
static bool bno08xTest()
{
  DEBUG_PRINT("BNO08xDeck test\n");
  return true;
}

void bno08xTask(void* arg)
{
  // systemWaitStart();
  // TickType_t xLastWakeTime;

  // // Write stuff to sensor if needed

  // xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelay(M2T(1));
    sh2_SensorValue_t sensor_data;

    if (wasReset()) {
      enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
    }

    got_data = 2;
    if (getSensorEvent(&sensor_data)) {
      got_data = 1;
      float qr = sensor_data.un.gameRotationVector.real;
      float qi = sensor_data.un.gameRotationVector.i;
      float qj = sensor_data.un.gameRotationVector.j;
      float qk = sensor_data.un.gameRotationVector.k;


      quaternionToEuler(qr, qi, qj, qk, &yaw, &pitch, &roll);
      
      // calculate r and s
      r_pos = l*(sinf(pitch));
      s_pos = -l*(sinf(roll)*cosf(pitch));

      // TODO: Calculate rdot_pos and sdot_pos and add those below

      // add r and s stuff to the pendulum struct
      pendulumMeasurement_t pendulum;
      pendulum.r_pos = r_pos;
      pendulum.s_pos = s_pos;
      pendulum.timestamp = usecTimestamp();
      estimatorEnqueuePendulum(&pendulum);
    }
  }
}


static const DeckDriver bno08xDriver = {
  .name = "bno08xDeck",
  .init = bno08xInit,
  .test = bno08xTest,
};

DECK_DRIVER(bno08xDriver);

LOG_GROUP_START(BNO08x)
LOG_ADD(LOG_UINT8, I2C_began, &I2C_began)
LOG_ADD(LOG_UINT8, reports_enabled, &reports_enabled)
LOG_ADD(LOG_UINT8, got_data, &got_data)
LOG_ADD(LOG_FLOAT, yaw, &yaw)
LOG_ADD(LOG_FLOAT, pitch, &pitch)
LOG_ADD(LOG_FLOAT, roll, &roll)
LOG_ADD(LOG_FLOAT, r_pos, &r_pos)
LOG_ADD(LOG_FLOAT, s_pos, &s_pos)
LOG_ADD(LOG_UINT8, acquired_data, &acquired_data[16])
LOG_ADD(LOG_INT32, status_log, &status_log)
LOG_GROUP_STOP(BNO08x)
