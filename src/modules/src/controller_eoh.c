#include "controller_eoh.h"
#include "stabilizer_types.h"
#include "power_distribution.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include <FreeRTOS.h>

// Sensor measurements
// - tof (from the z ranger on the flow deck)
static uint16_t tof_count = 0;
static float tof_distance = 0.0f;
// - flow (from the optical flow sensor on the flow deck)
static uint16_t flow_count = 0;
static float flow_dpixelx = 0.0f;
static float flow_dpixely = 0.0f;

// - pendulum (from the BNO08x connected to the pendulum/prototyping deck)
static uint16_t pendulum_count = 0;
static float r_pos = 0.0f;
static float s_pos = 0.0f;
static float rdot_pos = 0.0f;
static float sdot_pos = 0.0f;

// Setpoint
static float o_x_des = 0.0f;
static float o_y_des = 0.0f;
static float o_z_des = 0.0f;

// Input
static float tau_x = 0.0f;
static float tau_y = 0.0f;
static float tau_z = 0.0f;
static float f_z = 0.0f;

// Motor power command
static uint16_t m_1 = 0;
static uint16_t m_2 = 0;
static uint16_t m_3 = 0;
static uint16_t m_4 = 0;

// Measurements
static float n_x = 0.0f;
static float n_y = 0.0f;
static float r = 0.0f;
static float a_z = 0.0f;

// State
static float o_x = 0.0f;
static float o_y = 0.0f;
static float o_z = 0.0f;
static float psi = 0.0f;
static float theta = 0.0f;
static float phi = 0.0f;
static float v_x = 0.0f;
static float v_y = 0.0f;
static float v_z = 0.0f;
static float w_x = 0.0f;
static float w_y = 0.0f;
static float w_z = 0.0f;
static uint64_t pendulum_close_timestamp = 0;

// An example parameter
static bool use_observer = false;


void eohUpdateWithTOF(tofMeasurement_t *tof)
{
  tof_distance = tof->distance;
  tof_count++;
}

void eohUpdateWithFlow(flowMeasurement_t *flow)
{
  flow_dpixelx = flow->dpixelx;
  flow_dpixely = flow->dpixely;
  flow_count++;
}

void eohUpdateWithPendulum(pendulumMeasurement_t *pendulum)
{
  r_pos = pendulum->r_pos;
  s_pos = pendulum->s_pos;
  rdot_pos = pendulum->rdot_pos;
  sdot_pos = pendulum->sdot_pos;
  pendulum_count++;
}

void eohUpdateWithDistance(distanceMeasurement_t *meas)
{
  // If you have a loco positioning deck, this function will be called
  // each time a distance measurement is available. You will have to write
  // code to handle these measurements. These data are available:
  //
  //  meas->anchorId  uint8_t   id of anchor with respect to which distance was measured
  //  meas->x         float     x position of this anchor
  //  meas->y         float     y position of this anchor
  //  meas->z         float     z position of this anchor
  //  meas->distance  float     the measured distance
}

void eohUpdateWithPosition(positionMeasurement_t *meas)
{
  // This function will be called each time you send an external position
  // measurement (x, y, z) from the client, e.g., from a motion capture system.
  // You will have to write code to handle these measurements. These data are
  // available:
  //
  //  meas->x         float     x component of external position measurement
  //  meas->y         float     y component of external position measurement
  //  meas->z         float     z component of external position measurement
}

void eohUpdateWithPose(poseMeasurement_t *meas)
{
  // This function will be called each time you send an external "pose" measurement
  // (position as x, y, z and orientation as quaternion) from the client, e.g., from
  // a motion capture system. You will have to write code to handle these measurements.
  // These data are available:
  //
  //  meas->x         float     x component of external position measurement
  //  meas->y         float     y component of external position measurement
  //  meas->z         float     z component of external position measurement
  //  meas->quat.x    float     x component of quaternion from external orientation measurement
  //  meas->quat.y    float     y component of quaternion from external orientation measurement
  //  meas->quat.z    float     z component of quaternion from external orientation measurement
  //  meas->quat.w    float     w component of quaternion from external orientation measurement
}

void eohUpdateWithData(const struct EOHData* data)
{
  // This function will be called each time EOH-specific data are sent
  // from the client to the drone. You will have to write code to handle
  // these data. For the example EOHData struct, these data are:
  //
  //  data->x         float
  //  data->y         float
  //  data->z         float
  //
  // Exactly what "x", "y", and "z" mean in this context is up to you.
}


void controllerEOHInit(void)
{
  // Do nothing
}

bool controllerEOHTest(void)
{
  // Do nothing (test is always passed)
  return true;
}

void controllerEOH(control_t *control,
                     setpoint_t *setpoint,
                     const sensorData_t *sensors,
                     const state_t *state,
                     const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Whatever is in here executes at 500 Hz

    // Parse state (making sure to convert linear velocity from the world frame to the body frame)
    o_x = state->position.x;
    o_y = state->position.y;
    o_z = state->position.z;
    psi = radians(state->attitude.yaw);
    theta = - radians(state->attitude.pitch);
    phi = radians(state->attitude.roll);
    w_x = radians(sensors->gyro.x);
    w_y = radians(sensors->gyro.y);
    w_z = radians(sensors->gyro.z);
    v_x = state->velocity.x*cosf(psi)*cosf(theta) + state->velocity.y*sinf(psi)*cosf(theta) - state->velocity.z*sinf(theta);
    v_y = state->velocity.x*(sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi)) + state->velocity.y*(sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi)) + state->velocity.z*sinf(phi)*cosf(theta);
    v_z = state->velocity.x*(sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi)) + state->velocity.y*(-sinf(phi)*cosf(psi) + sinf(psi)*sinf(theta)*cosf(phi)) + state->velocity.z*cosf(phi)*cosf(theta);

    // Parse setpoint
    o_x_des = setpoint->position.x;
    o_y_des = setpoint->position.y;
    o_z_des = setpoint->position.z;

    // Parse measurements
    n_x = flow_dpixelx;
    n_y = flow_dpixely;
    r = tof_distance;
    a_z = 9.81f * sensors->acc.z;

    if (setpoint->mode.z == modeDisable) {
      // If there is no desired position, then all
      // motor power commands should be zero

      powerSet(0, 0, 0, 0);
    } else {
      // Otherwise, motor power commands should be
      // chosen by the controller
      if (r_pos*r_pos > (0.05f*0.05f)) {
        pendulum_close_timestamp = usecTimestamp();
      }

      bool do_controller = (usecTimestamp() - pendulum_close_timestamp) > 5000000;

      if (!do_controller) {
        tau_x = 0.00264575f * (o_y - o_y_des) -0.00667388f * phi + 0.00209759f * v_y -0.00110243f * w_x;
        tau_y = -0.00223607f * (o_x - o_x_des) -0.00654857f * theta -0.00194559f * v_x -0.00108695f * w_y;
        tau_z = -0.00100000f * psi -0.00102777f * w_z;
        f_z = -0.21447611f * (o_z - o_z_des) -0.18271100f * v_z + 0.35f;
      } else {
      // FIXME
        tau_x = -0.00707107f * (o_y - o_y_des) -0.00699189f * v_y -0.02385166f * phi -0.00132248f * w_x -0.16967033f * s_pos -0.03065506f * sdot_pos;
        tau_y = 0.00707107f * (o_x - o_x_des) + 0.00699388f * v_x -0.02391362f * theta -0.00132862f * w_y + 0.16992423f * r_pos + 0.03070090f * rdot_pos;
        tau_z = -0.00100000f * psi -0.00102029f * w_z;
        f_z = -0.31622777f * (o_z - o_z_des) -0.34857350f * v_z + 0.46354000f;
      }

      // FIXME
      m_1 = limitUint16( -3622138.5f * tau_x -3622138.5f * tau_y -27654867.3f * tau_z + 123152.7f * f_z );
      m_2 = limitUint16( -3622138.5f * tau_x + 3622138.5f * tau_y + 27654867.3f * tau_z + 123152.7f * f_z );
      m_3 = limitUint16( 3622138.5f * tau_x + 3622138.5f * tau_y -27654867.3f * tau_z + 123152.7f * f_z );
      m_4 = limitUint16( 3622138.5f * tau_x -3622138.5f * tau_y + 27654867.3f * tau_z + 123152.7f * f_z );
      
      // Apply motor power commands
      powerSet(m_1, m_2, m_3, m_4);
    }

  // if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
  //   // Whatever is in here executes at 100 Hz

  // }

  // By default, we do nothing (set all motor power commands to zero)
  // powerSet(0, 0, 0, 0);
}
}

//              1234567890123456789012345678 <-- max total length
//              group   .name
LOG_GROUP_START(eohlog)
LOG_ADD(LOG_UINT16,         num_tof,                &tof_count)
LOG_ADD(LOG_UINT16,         num_flow,               &flow_count)
LOG_ADD(LOG_FLOAT,          r_pos,                  &r_pos)
LOG_ADD(LOG_FLOAT,          s_pos,                  &s_pos)
LOG_ADD(LOG_FLOAT,          rdot_pos,               &rdot_pos)
LOG_ADD(LOG_FLOAT,          sdot_pos,               &sdot_pos)
LOG_ADD(LOG_UINT16,         m_1,                     &m_1)
LOG_ADD(LOG_UINT16,         m_2,                     &m_2)
LOG_ADD(LOG_UINT16,         m_3,                     &m_3)
LOG_ADD(LOG_UINT16,         m_4,                     &m_4)
LOG_GROUP_STOP(eohlog)

//                1234567890123456789012345678 <-- max total length
//                group   .name
PARAM_GROUP_START(eohpar)
PARAM_ADD(PARAM_UINT8,     use_observer,            &use_observer)
PARAM_GROUP_STOP(eohpar)