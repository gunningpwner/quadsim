/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 1995 - 2024 SEGGER Microcontroller GmbH             *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
*       SEGGER SystemView * Real-time application analysis           *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* SEGGER strongly recommends to not make any changes                 *
* to or modify the source code of this software in order to stay     *
* compatible with the SystemView and RTT protocol, and J-Link.       *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* condition is met:                                                  *
*                                                                    *
* o Redistributions of source code must retain the above copyright   *
*   notice, this condition and the following disclaimer.             *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED. IN NO EVENT SHALL SEGGER Microcontroller BE LIABLE FOR *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************
-------------------------- END-OF-HEADER -----------------------------

File    : SEGGER_SYSVIEW_Config_NoOS.c
Purpose : Sample setup configuration of SystemView without an OS.
Revision: $Rev: 9599 $
*/
#include "SEGGER_SYSVIEW.h"
#include "SEGGER_SYSVIEW_Conf.h"
#include <string.h>
// SystemcoreClock can be used in most CMSIS compatible projects.
// In non-CMSIS projects define SYSVIEW_CPU_FREQ.
extern unsigned int SystemCoreClock;

/*********************************************************************
*
*       Defines, configurable
*
**********************************************************************
*/
// The application name to be displayed in SystemViewer
#define SYSVIEW_APP_NAME        "Tobe"

// The target device name
#define SYSVIEW_DEVICE_NAME     "Cortex-M4"

// Frequency of the timestamp. Must match SEGGER_SYSVIEW_Conf.h
#define SYSVIEW_TIMESTAMP_FREQ  (SystemCoreClock)

// System Frequency. SystemcoreClock is used in most CMSIS compatible projects.
#define SYSVIEW_CPU_FREQ        (SystemCoreClock)

// The lowest RAM address used for IDs (pointers)
#define SYSVIEW_RAM_BASE        (0x10000000)

// Define as 1 if the Cortex-M cycle counter is used as SystemView timestamp. Must match SEGGER_SYSVIEW_Conf.h
#ifndef   USE_CYCCNT_TIMESTAMP
  #define USE_CYCCNT_TIMESTAMP    1
#endif

// Define as 1 if the Cortex-M cycle counter is used and there might be no debugger attached while recording.
#ifndef   ENABLE_DWT_CYCCNT
  #define ENABLE_DWT_CYCCNT       (USE_CYCCNT_TIMESTAMP & SEGGER_SYSVIEW_POST_MORTEM_MODE)
#endif

/*********************************************************************
*
*       Defines, fixed
*
**********************************************************************
*/
#define DEMCR                     (*(volatile unsigned long*) (0xE000EDFCuL))   // Debug Exception and Monitor Control Register
#define TRACEENA_BIT              (1uL << 24)                                   // Trace enable bit
#define DWT_CTRL                  (*(volatile unsigned long*) (0xE0001000uL))   // DWT Control Register
#define NOCYCCNT_BIT              (1uL << 25)                                   // Cycle counter support bit
#define CYCCNTENA_BIT             (1uL << 0)                                    // Cycle counter enable bit

/********************************************************************* 
*
*       _cbSendSystemDesc()
*
*  Function description
*    Sends SystemView description strings.
*/
static const SEGGER_SYSVIEW_DATA_REGISTER buffer_reg_data = {
    .ID            = SENSOR_BUFFER_DATA_ID,
    .DataType      = SEGGER_SYSVIEW_TYPE_U32, // Unsigned 32-bit int
    .Offset        = 0,
    .RangeMin      = 0,     
    .RangeMax      = 0,     // Leaving Min/Max at 0 tells SystemView to auto-scale the Y-axis
    .ScalingFactor = 1.0f,  // IMPORTANT: This must be non-zero or the graph won't draw!
    .sName         = "Sensor Buffer",
    .sUnit         = "Measurements"
};

static const SEGGER_SYSVIEW_DATA_REGISTER motor_1_data = {
    .ID            = MOTOR_1_DATA_ID,
    .DataType      = SEGGER_SYSVIEW_TYPE_U32,
    .Offset        = 0,
    .RangeMin      = 0,     
    .RangeMax      = 3000,     // Leaving Min/Max at 0 tells SystemView to auto-scale the Y-axis
    .ScalingFactor = 1.0f,  // IMPORTANT: This must be non-zero or the graph won't draw!
    .sName         = "Motor 1",
    .sUnit         = "Throttle"
};

static const SEGGER_SYSVIEW_DATA_REGISTER motor_2_data = {
    .ID            = MOTOR_2_DATA_ID,
    .DataType      = SEGGER_SYSVIEW_TYPE_U32,
    .Offset        = 0,
    .RangeMin      = 0,     
    .RangeMax      = 3000,
    .ScalingFactor = 1.0f,
    .sName         = "Motor 2",
    .sUnit         = "Throttle"
};

static const SEGGER_SYSVIEW_DATA_REGISTER motor_3_data = {
    .ID            = MOTOR_3_DATA_ID,
    .DataType      = SEGGER_SYSVIEW_TYPE_U32,
    .Offset        = 0,
    .RangeMin      = 0,     
    .RangeMax      = 3000,
    .ScalingFactor = 1.0f,
    .sName         = "Motor 3",
    .sUnit         = "Throttle"
};

static const SEGGER_SYSVIEW_DATA_REGISTER motor_4_data = {
    .ID            = MOTOR_4_DATA_ID,
    .DataType      = SEGGER_SYSVIEW_TYPE_U32,
    .Offset        = 0,
    .RangeMin      = 0,     
    .RangeMax      = 3000,
    .ScalingFactor = 1.0f,
    .sName         = "Motor 4",
    .sUnit         = "Throttle"
};
static const SEGGER_SYSVIEW_DATA_REGISTER valid_rpm = {
    .ID            = VALID_RPM_DATA_ID,
    .DataType      = SEGGER_SYSVIEW_TYPE_U32,
    .Offset        = 0,
    .RangeMin      = 0,     
    .RangeMax      = 4,
    .ScalingFactor = 1.0f,
    .sName         = "Valid RPM",
    .sUnit         = "aaa"
};


static void _cbSendSystemDesc(void) {
  SEGGER_SYSVIEW_SendSysDesc("N="SYSVIEW_APP_NAME",D="SYSVIEW_DEVICE_NAME);
  SEGGER_SYSVIEW_SendSysDesc("I#15=SysTick");
  SEGGER_SYSVIEW_SendSysDesc("I#44=IMU/Compass");
  SEGGER_SYSVIEW_SendSysDesc("I#72=runFastLoop");
  SEGGER_SYSVIEW_SendSysDesc("I#55=USART3_CRSF");
  SEGGER_SYSVIEW_SendSysDesc("I#28=DMA1_S1_CRSF_RX");
  SEGGER_SYSVIEW_SendSysDesc("I#68=UART4_GPS");
  SEGGER_SYSVIEW_SendSysDesc("I#29=DMA1_S2_GPS_RX");
  SEGGER_SYSVIEW_SendSysDesc("I#47=ProcessCompass");
  SEGGER_SYSVIEW_SendSysDesc("I#31=DMA1_Stream4_IRQn");
  SEGGER_SYSVIEW_SendSysDesc("I#75=DMA2_Stream3_IRQ");
  SEGGER_SYSVIEW_SendSysDesc("I#76=DMA2_Stream4_IRQ");
  SEGGER_SYSVIEW_SendSysDesc("I#83=OTG_FS");
  SEGGER_SYSVIEW_TASKINFO eskf_info;
  memset(&eskf_info, 0, sizeof(eskf_info));
  eskf_info.TaskID = ESKF_TASK; 
  eskf_info.sName  = "ESKF_TASK";
  SEGGER_SYSVIEW_SendTaskInfo(&eskf_info);




  SEGGER_SYSVIEW_NameMarker(ESKF_IMU_MARKER, "ESKF_IMU");
  SEGGER_SYSVIEW_NameMarker(ESKF_GPS_MARKER, "ESKF_GPS");
  SEGGER_SYSVIEW_NameMarker(ESKF_MAG_MARKER, "ESKF_MAG");
  


  SEGGER_SYSVIEW_RegisterData((SEGGER_SYSVIEW_DATA_REGISTER*)&buffer_reg_data);
  SEGGER_SYSVIEW_RegisterData((SEGGER_SYSVIEW_DATA_REGISTER*)&motor_1_data);
  SEGGER_SYSVIEW_RegisterData((SEGGER_SYSVIEW_DATA_REGISTER*)&motor_2_data);
  SEGGER_SYSVIEW_RegisterData((SEGGER_SYSVIEW_DATA_REGISTER*)&motor_3_data);
  SEGGER_SYSVIEW_RegisterData((SEGGER_SYSVIEW_DATA_REGISTER*)&motor_4_data);
  SEGGER_SYSVIEW_RegisterData((SEGGER_SYSVIEW_DATA_REGISTER*)&valid_rpm);

}

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/
void SEGGER_SYSVIEW_Conf(void) {
#if USE_CYCCNT_TIMESTAMP
#if ENABLE_DWT_CYCCNT
  //
  // If no debugger is connected, the DWT must be enabled by the application
  //
  if ((DEMCR & TRACEENA_BIT) == 0) {
    DEMCR |= TRACEENA_BIT;
  }
#endif
  //
  //  The cycle counter must be activated in order
  //  to use time related functions.
  //
  if ((DWT_CTRL & NOCYCCNT_BIT) == 0) {       // Cycle counter supported?
    if ((DWT_CTRL & CYCCNTENA_BIT) == 0) {    // Cycle counter not enabled?
      DWT_CTRL |= CYCCNTENA_BIT;              // Enable Cycle counter
    }
  }
#endif
  SEGGER_SYSVIEW_Init(SYSVIEW_TIMESTAMP_FREQ, SYSVIEW_CPU_FREQ, 
                      0, _cbSendSystemDesc);
  SEGGER_SYSVIEW_SetRAMBase(SYSVIEW_RAM_BASE);
}

/*************************** End of file ****************************/
