#ifndef __CONTROLLER_EOH_H__
#define __CONTROLLER_EOH_H__

#include "stabilizer_types.h"

// An example struct to hold EOH-specific data sent from client to drone
struct EOHData
{
  float x;
  float y;
  float z;
} __attribute__((packed));

void controllerEOHInit(void);
bool controllerEOHTest(void);
void controllerEOH(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

// Functions to receive measurements
void eohUpdateWithTOF(tofMeasurement_t *tof);
void eohUpdateWithFlow(flowMeasurement_t *flow);
void eohUpdateWithPendulum(pendulumMeasurement_t *pendulum);
void eohUpdateWithDistance(distanceMeasurement_t *meas);
void eohUpdateWithPosition(positionMeasurement_t *meas);
void eohUpdateWithPose(poseMeasurement_t *meas);

// Functions to receive EOH-specific data sent from client to drone
void eohUpdateWithData(const struct EOHData* data);


#endif //__CONTROLLER_EOH_H__
