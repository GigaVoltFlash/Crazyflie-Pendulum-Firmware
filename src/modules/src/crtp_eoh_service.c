#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "crtp.h"
#include "crtp_eoh_service.h"
#include "controller_eoh.h"
#include "log.h"

static void eohserviceCrtpCB(CRTPPacket* pk);

static bool isInit = false;
static uint16_t tickOfLastPacket;


void eohserviceInit()
{
  if (isInit) {
    return;
  }

  crtpRegisterPortCB(CRTP_PORT_EOH, eohserviceCrtpCB);
  isInit = true;
}

static void eohserviceCrtpCB(CRTPPacket* pk)
{
  // We assume that packets always have the same structure. One way
  // to handle packets with different structure would be to write a
  // case-based handler that switches on pk->channel (e.g., see the
  // function locSrvCrtpCB in crtp_localization_service.c).
  const struct EOHData* data = (const struct EOHData*)pk->data;
  eohUpdateWithData(data);
  tickOfLastPacket = xTaskGetTickCount();
}

//              1234567890123456789012345678 <-- max total length
//              group   .name
LOG_GROUP_START(eohcom)
LOG_ADD(LOG_UINT16,      tick,                  &tickOfLastPacket)
LOG_GROUP_STOP(eohcom)
