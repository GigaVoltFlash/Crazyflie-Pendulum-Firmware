#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "crtp.h"
#include "crtp_ae483_service.h"
#include "controller_ae483.h"
#include "log.h"

static void ae483serviceCrtpCB(CRTPPacket* pk);

static bool isInit = false;
static uint16_t tickOfLastPacket;


void ae483serviceInit()
{
  if (isInit) {
    return;
  }

  crtpRegisterPortCB(CRTP_PORT_AE483, ae483serviceCrtpCB);
  isInit = true;
}

static void ae483serviceCrtpCB(CRTPPacket* pk)
{
  // We assume that packets always have the same structure. One way
  // to handle packets with different structure would be to write a
  // case-based handler that switches on pk->channel (e.g., see the
  // function locSrvCrtpCB in crtp_localization_service.c).
  const struct AE483Data* data = (const struct AE483Data*)pk->data;
  ae483UpdateWithData(data);
  tickOfLastPacket = xTaskGetTickCount();
}

//              1234567890123456789012345678 <-- max total length
//              group   .name
LOG_GROUP_START(ae483com)
LOG_ADD(LOG_UINT16,      tick,                  &tickOfLastPacket)
LOG_GROUP_STOP(ae483com)
