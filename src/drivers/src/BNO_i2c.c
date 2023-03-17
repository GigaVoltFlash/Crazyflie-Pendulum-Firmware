
/* 
 * BNO I2C interface
 * This is to get data from the BNO over i2c but using the I2c methods
 * 
*/

#include "BNO_i2c.h"

int8_t _int_pin, _reset_pin;

sh2_SensorValue_t *_sensor_value = NULL;
sh2_Hal_t _HAL; ///< The struct representing the SH2 Hardware Abstraction Layer
sh2_ProductIds_t prodIds; ///< The product IDs returned by the sensor


bool _reset_occurred = false;

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param  sensor_id
 *            The unique ID to differentiate the sensors from others
 *    @return True if initialization was successful, otherwise false.
 */
bool begin_I2C(uint8_t i2c_address, int32_t sensor_id) {

  _HAL.open = i2chal_open;
  _HAL.close = i2chal_close;
  _HAL.read = i2chal_read;
  _HAL.write = i2chal_write;
  _HAL.getTimeUs = hal_getTimeUs;

  _reset_occurred = true;
  // return true;
  return _init(sensor_id);
}


extern int status_log;
/*!  @brief Initializer for post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */

bool _init(int32_t sensor_id) {
  int status;

  // hardwareReset();

  // Open SH2 interface (also registers non-sensor event handler.)
  status = sh2_open(&_HAL, hal_callback, NULL);
  if (status != SH2_OK) {
    status_log = status;
    return false;
  }

  // Check connection partially by getting the product id's
  memset(&prodIds, 0, sizeof(prodIds));
  status = sh2_getProdIds(&prodIds);
  if (status != SH2_OK) {
    status_log = -status;
    return false;
  }

  // Register sensor listener
  sh2_setSensorCallback(sensorHandler, NULL);

  return true;
}


/**
 * @brief Reset the device using the Reset pin
 *
 */
// void hardwareReset(void) { hal_hardwareReset(); }

/**
 * @brief Check if a reset has occured
 *
 * @return true: a reset has occured false: no reset has occoured
 */
bool wasReset(void) {
  bool x = _reset_occurred;
  _reset_occurred = false;

  return x;
}

/**
 * @brief Fill the given sensor value object with a new report
 *
 * @param value Pointer to an sh2_SensorValue_t struct to fil
 * @return true: The report object was filled with a new report
 * @return false: No new report available to fill
 */
bool getSensorEvent(sh2_SensorValue_t *value) {
  _sensor_value = value;

  value->timestamp = 0;

  sh2_service();

  if (value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV) {
    // no new events
    return false;
  }

  return true;
}

/**
 * @brief Enable the given report type
 *
 * @param sensorId The report ID to enable
 * @param interval_us The update interval for reports to be generated, in
 * microseconds
 * @return true: success false: failure
 */
bool enableReport(sh2_SensorId_t sensorId,
                                   uint32_t interval_us) {
  static sh2_SensorConfig_t config;

  // These sensor options are disabled or not used in most cases
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = 0;

  config.reportInterval_us = interval_us;
  int status = sh2_setSensorConfig(sensorId, &config);

  if (status != SH2_OK) {
    return false;
  }

  return true;
}


/**************************************** I2C interface
 * ***********************************************************/

int i2chal_open(sh2_Hal_t *self) {
  // Serial.println("I2C HAL open");
  uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
  bool success = false;
  for (uint8_t attempts = 0; attempts < 5; attempts++) {
    if (i2cdevWrite(I2C1_DEV, BNO08x_I2CADDR_DEFAULT, 5, softreset_pkt)) {
      success = true;
      break;
    }
    vTaskDelay(0.03 * configTICK_RATE_HZ);
  }
  if (!success)
    return -1;
  vTaskDelay(0.3 * configTICK_RATE_HZ);
  return 0;
}

void i2chal_close(sh2_Hal_t *self) {
  // Serial.println("I2C HAL close");
}

int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us) {
  // Serial.println("I2C HAL read");

  // uint8_t *pBufferOrig = pBuffer;

  uint8_t header[4];
  if (!i2cdevRead(I2C1_DEV, BNO08x_I2CADDR_DEFAULT, 4, header)) {
    return 0;
  }

  // Determine amount to read
  uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
  // Unset the "continue" bit
  packet_size &= ~0x8000;

  /*
  Serial.print("Read SHTP header. ");
  Serial.print("Packet size: ");
  Serial.print(packet_size);
  Serial.print(" & buffer size: ");
  Serial.println(len);
  */

  size_t i2c_buffer_max = MAX_BUFFER_SIZE;

  if (packet_size > len) {
    // packet wouldn't fit in our buffer
    return 0;
  }
  // the number of non-header bytes to read
  uint16_t cargo_remaining = packet_size;
  uint8_t i2c_buffer[i2c_buffer_max];
  uint16_t read_size;
  uint16_t cargo_read_amount = 0;
  bool first_read = true;

  while (cargo_remaining > 0) {
    if (first_read) {
      read_size = int_min(i2c_buffer_max, (size_t)cargo_remaining);
    } else {
      read_size = int_min(i2c_buffer_max, (size_t)cargo_remaining + 4);
    }

    // Serial.print("Reading from I2C: "); Serial.println(read_size);
    // Serial.print("Remaining to read: "); Serial.println(cargo_remaining);

    if (!i2cdevRead(I2C1_DEV, BNO08x_I2CADDR_DEFAULT, read_size, i2c_buffer)) {
        return 0;
    }

    if (first_read) {
      // The first time we're saving the "original" header, so include it in the
      // cargo count
      cargo_read_amount = read_size;
      memcpy(pBuffer, i2c_buffer, cargo_read_amount);
      first_read = false;
    } else {
      // this is not the first read, so copy from 4 bytes after the beginning of
      // the i2c buffer to skip the header included with every new i2c read and
      // don't include the header in the amount of cargo read
      cargo_read_amount = read_size - 4;
      memcpy(pBuffer, i2c_buffer + 4, cargo_read_amount);
    }
    // advance our pointer by the amount of cargo read
    pBuffer += cargo_read_amount;
    // mark the cargo as received
    cargo_remaining -= cargo_read_amount;
  }

  /*
  for (int i=0; i<packet_size; i++) {
    Serial.print(pBufferOrig[i], HEX);
    Serial.print(", ");
    if (i % 16 == 15) Serial.println();
  }
  Serial.println();
  */

  return packet_size;
}

int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  size_t i2c_buffer_max = MAX_BUFFER_SIZE;

  /*
  Serial.print("I2C HAL write packet size: ");
  Serial.print(len);
  Serial.print(" & max buffer size: ");
  Serial.println(i2c_buffer_max);
  */

  uint16_t write_size = int_min(i2c_buffer_max, len);
  if (!i2cdevWrite(I2C1_DEV, BNO08x_I2CADDR_DEFAULT, write_size, pBuffer)) {
    return 0;
  }

  return write_size;
}


/**************************************** HAL interface
 * ***********************************************************/

uint32_t hal_getTimeUs(sh2_Hal_t *self) {
  uint32_t t = xTaskGetTickCount() * portTICK_RATE_MS * 1000;
  // Serial.printf("I2C HAL get time: %d\n", t);
  return t;
}

void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
  // If we see a reset, set a flag so that sensors will be reconfigured.
  if (pEvent->eventId == SH2_RESET) {
    // Serial.println("Reset!");
    _reset_occurred = true;
  }
}

// Handle sensor events.
void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {
  int rc;

  // Serial.println("Got an event!");

  rc = sh2_decodeSensorEvent(_sensor_value, event);
  if (rc != SH2_OK) {
    _sensor_value->timestamp = 0;
    return;
  }
}

/**************************************** Quaternion to Euler
 * ***********************************************************/
void quaternionToEuler(float qr, float qi, float qj, float qk, float *yaw, float *pitch, float *roll) {
    float sqr = qr * qr;
    float sqi = qi * qi;
    float sqj = qj * qj;
    float sqk = qk * qk;

    *yaw =
        atan2(2.0 * (double)(qi * qj + qk * qr), (double)(sqi - sqj - sqk + sqr));
    *pitch = asin(-2.0 * (double)(qi * qk - qj * qr) / (double)(sqi + sqj + sqk + sqr));
    *roll =
        atan2(2.0 * (double)(qj * qk + qi * qr), (double)(-sqi - sqj + sqk + sqr));

    return;
}