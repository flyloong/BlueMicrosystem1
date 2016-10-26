/**
  ******************************************************************************
  * @file    osx_bms_config.h
  * @author  Central LAB
  * @version V3.0.0
  * @date    01-June-2016
  * @brief   osx Bluemicrosystem configuration
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OSX_BMS_CONFIG_H
#define __OSX_BMS_CONFIG_H

/* Exported define ------------------------------------------------------------*/

/* For reading the oxMotion licenses from osx_license.h files */
/*#define OSX_BMS_LICENSE_H_FILE */

/* For enabling MotionCP integration */
#define OSX_BMS_MOTIONCP

/* For enabling MotionAR integration */
#define OSX_BMS_MOTIONAR

/* For enabling MotionGR integration */
#define OSX_BMS_MOTIONGR

/* For enabling MotionPM integration */
#define OSX_BMS_MOTIONPM

/* Define the BlueMicrosystem MAC address, otherwise it will use a random MAC */
//#define MAC_BLUEMS 0xFF, 0xEE, 0xDD, 0xAA, 0xAA, 0xAA

/* Define The transmission interval in Multiple of 10ms for quaternions*/
#define QUAT_UPDATE_MUL_10MS 3

/* Define How Many quaterions you want to trasmit (from 1 to 3) */
#define SEND_N_QUATERNIONS 3

/* IMPORTANT 
The Sensors fusion runs at 100Hz so like MAXIMUM it possible to send:
1 quaternion every 10ms
2 quaternions every 20ms
3 quaternions every 30ms

if QUAT_UPDATE_MUL_10MS!=3, then SEND_N_QUATERNIONS must be ==1
*/

/*************** Debug Defines ******************/
#ifdef STM32_NUCLEO
/* For enabling the printf on UART */
#define OSX_BMS_ENABLE_PRINTF
#endif /* STM32_NUCLEO */

/* For enabling connection and notification subscriptions debug */
#define OSX_BMS_DEBUG_CONNECTION

/* For enabling trasmission for notified services (except for quaternions) */
#define OSX_BMS_DEBUG_NOTIFY_TRAMISSION


/*************** Don't Change the following defines *************/

/* Package Version */
#define OSX_BMS_VERSION_MAJOR '3'
#define OSX_BMS_VERSION_MINOR '0'
#define OSX_BMS_VERSION_PATCH '0'

/* Define the BlueMicrosystem Name MUST be 7 char long */
#define NAME_BLUEMS 'B','M','1','V',OSX_BMS_VERSION_MAJOR,OSX_BMS_VERSION_MINOR,OSX_BMS_VERSION_PATCH

/* Package Name */
#define OSX_BMS_PACKAGENAME "BLUEMICROSYSTEM1"

/* Board/BlueNRG FW OTA Postion */
#define OSX_BMS_OTA_ADDRESS_START  0x08040008

/* Board  FW OTA Magic Number Position */
#define OSX_BMS_OTA_MAGIC_NUM_POS  0x08040000

/* Board  FW OTA Magic Number */
#define OSX_BMS_OTA_MAGIC_NUM 0xDEADBEEF

/* 240Kbytes Max Program Size */
#define OSX_BMS_MAX_PROG_SIZE (0x3FFFF-0x3FFF)

#ifdef OSX_BMS_ENABLE_PRINTF
#define OSX_BMS_PRINTF(...) printf(__VA_ARGS__)
#else /* OSX_BMS_ENABLE_PRINTF */
#define OSX_BMS_PRINTF(...) 
#endif /* OSX_BMS_ENABLE_PRINTF */

/* STM32 Unique ID */
#ifdef USE_STM32F4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7A10)
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7590)
#endif /* USE_STM32L4XX_NUCLEO */

/* STM32 MCU_ID */
#define STM32_MCU_ID ((uint32_t *)0xE0042000)

/* Control Section */

#if ((SEND_N_QUATERNIONS<1) || (SEND_N_QUATERNIONS>3))
  #error "SEND_N_QUATERNIONS could be only 1,2 or 3"
#endif

#if ((QUAT_UPDATE_MUL_10MS!=3) && (SEND_N_QUATERNIONS!=1))
  #error "If QUAT_UPDATE_MUL_10MS!=3 then SEND_N_QUATERNIONS must be = 1"
#endif

#endif /* __OSX_BMS_CONFIG_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
