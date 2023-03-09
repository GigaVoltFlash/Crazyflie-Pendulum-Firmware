/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * vcp_esc_passthrough.c - Module to handle 4way passthough interface
 */

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "queuemonitor.h"

#include "system.h"
#include "config.h"
#include "static_mem.h"
#include "info.h"
#include "version.h"
#include "usb.h"
#include "motors.h"
#include "serial_4way.h"
#include "uart_syslink.h"
#include "sensors.h"

static TaskHandle_t passthroughTaskHandle;
STATIC_MEM_TASK_ALLOC(passthroughTask, PASSTHROUGH_TASK_STACKSIZE);

static bool isInit;

// Passthorugh queues to handle VCP data.
static xQueueHandle  ptRxQueue;
STATIC_MEM_QUEUE_ALLOC(ptRxQueue, 512, sizeof(uint8_t));
static xQueueHandle  ptTxQueue;
STATIC_MEM_QUEUE_ALLOC(ptTxQueue, 512, sizeof(uint8_t));

void passthroughTask(void *param);

void passthroughInit()
{
  if(isInit)
    return;

  ptRxQueue = STATIC_MEM_QUEUE_CREATE(ptRxQueue);
  DEBUG_QUEUE_MONITOR_REGISTER(ptRxQueue);
  ptTxQueue = STATIC_MEM_QUEUE_CREATE(ptTxQueue);
  DEBUG_QUEUE_MONITOR_REGISTER(ptRxQueue);

  passthroughTaskHandle = STATIC_MEM_TASK_CREATE(passthroughTask, passthroughTask, PASSTHROUGH_TASK_NAME, NULL, PASSTHROUGH_TASK_PRI);
}

void passthroughEnableFromISR()
{
  BaseType_t xHigherPriorityTaskWoken;
  vTaskNotifyGiveFromISR(passthroughTaskHandle, &xHigherPriorityTaskWoken);
}

void passthroughVcpRxSendFromISR(uint8_t Ch)
{
  BaseType_t xHigherPriorityTaskWoken;

  ASSERT(xQueueSendFromISR(ptRxQueue, &Ch, &xHigherPriorityTaskWoken) == pdTRUE);
}

void passthroughVcpRxSendBlock(uint8_t Ch)
{
  ASSERT(xQueueSend(ptRxQueue, &Ch, portMAX_DELAY) == pdTRUE);
}

int passthroughVcpRxReceive(uint8_t* receiveChPtr)
{
  ASSERT(receiveChPtr);
  return xQueueReceive(ptRxQueue, receiveChPtr, 0);
}

int passthroughVcpRxReceiveBlock(uint8_t* receiveChPtr)
{
  ASSERT(receiveChPtr);
  return xQueueReceive(ptRxQueue, receiveChPtr, portMAX_DELAY);
}

void passthroughVcpTxSend(uint8_t Ch)
{
  ASSERT(xQueueSend(ptTxQueue, &Ch, 0) == pdTRUE);
}

int  passthroughVcpTxReceiveFromISR(uint8_t* receiveChPtr)
{
  BaseType_t xHigherPriorityTaskWoken;
  return xQueueReceiveFromISR(ptTxQueue, receiveChPtr, &xHigherPriorityTaskWoken);
}


void passthroughTask(void *param)
{
  systemWaitStart();

  while (true)
  {
    // Wait for interface to be activated, typically when ACM or COM port control message is sent
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // ESC 1-wire interface is bit-banging so some interrupts must be suspended
    uartslkPauseRx();
    sensorsSuspend();

    // Suspend motor signal output and init them as normal GPIO
    esc4wayInit();
    // Run the 4way process
    esc4wayProcess();

    // After 4way process is done resume interrupts
    uartslkResumeRx();
    sensorsResume();

    // Clear any notifications that was queued during 4way process.
    ulTaskNotifyValueClear(NULL, 0xFFFFFFFF);
  }
}

