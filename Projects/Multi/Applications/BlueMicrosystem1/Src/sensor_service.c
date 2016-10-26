/**
  ******************************************************************************
  * @file    sensor_service.c
  * @author  Central LAB
  * @version V3.0.0
  * @date    01-June-2016
  * @brief   Add 4 bluetooth services using vendor specific profiles.
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
#include <stdio.h>
#include "TargetFeatures.h"
#include "main.h"
#include "LicenseManager.h"
#include "sensor_service.h"
#include "console.h"
#include "HWAdvanceFeatures.h"
#include "bluenrg_utils.h"
#include "bluenrg_l2cap_aci.h"
#include "uuid_ble_service.h"
#include "OTA.h"

/* Exported variables ---------------------------------------------------------*/
int connected = FALSE;
uint8_t set_connectable = TRUE;

/* Imported Variables -------------------------------------------------------------*/
extern osxMFX_calibFactor magOffset;

extern uint32_t ConnectionBleStatus;

#ifdef OSX_BMS_MOTIONAR
extern osx_MAR_output_t ActivityCode;
#endif /* OSX_BMS_MOTIONAR */

#ifdef OSX_BMS_MOTIONCP
extern osx_MCP_output_t CarryPositionCode;
#endif /* OSX_BMS_MOTIONCP */

#ifdef OSX_BMS_MOTIONGR
extern osx_MGR_output_t GestureRecognitionCode;
#endif /* OSX_BMS_MOTIONGR */

#ifdef OSX_BMS_MOTIONPM
extern osx_MPM_output_t PM_DataOUT;
#endif /* OSX_BMS_MOTIONPM */

extern uint32_t ForceReCalibration;
extern uint32_t FirstConnectionConfig;

extern TIM_HandleTypeDef    TimCCHandle;
extern TIM_HandleTypeDef    TimEnvHandle;

#ifndef STM32_NUCLEO
  #if defined (__IAR_SYSTEMS_ICC__)
    extern uint32_t RestartInBootLoaderMode;
  #elif defined (__CC_ARM)
    extern uint32_t *RestartInBootLoaderMode;
  #elif defined (__GNUC__)
    extern uint32_t RestartInBootLoaderMode;
  #else
    #error "Toolchain not supported"
  #endif
#endif /* STM32_NUCLEO */

/* Private variables ------------------------------------------------------------*/
static uint16_t HWServW2STHandle;
static uint16_t EnvironmentalCharHandle;
static uint16_t AccGyroMagCharHandle;
static uint16_t AccEventCharHandle;
static uint16_t LedCharHandle;

static uint16_t SWServW2STHandle;
static uint16_t QuaternionsCharHandle;

#ifdef OSX_BMS_MOTIONAR
static uint16_t ActivityRecCharHandle;
#endif /* OSX_BMS_MOTIONAR */

#ifdef OSX_BMS_MOTIONCP
static uint16_t CarryPosRecCharHandle;
#endif /* OSX_BMS_MOTIONCP */

#ifdef OSX_BMS_MOTIONGR
static uint16_t GestureRecCharHandle;
#endif /* OSX_BMS_MOTIONGR */

#ifdef OSX_BMS_MOTIONPM
static uint16_t AccPedoCharHandle;
#endif /* OSX_BMS_MOTIONPM */

static uint16_t ConfigServW2STHandle;
static uint16_t ConfigCharHandle;

static uint16_t ConsoleW2STHandle;
static uint16_t TermCharHandle;
static uint16_t StdErrCharHandle;

static uint8_t LastStderrBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastStderrLen;
static uint8_t LastTermBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastTermLen;

static uint8_t  EnvironmentalCharSize=2; /* Size for Environmental BLE characteristic */

static uint32_t SizeOfUpdateBlueFW=0;

static uint16_t connection_handle = 0;

/* Private functions ------------------------------------------------------------*/
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
static void GAP_DisconnectionComplete_CB(void);
static uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length);
static void ConfigCommandParsing(uint8_t * att_data, uint8_t data_length);

/* Private define ------------------------------------------------------------*/

#ifdef ACC_BLUENRG_CONGESTION
#define ACI_GATT_UPDATE_CHAR_VALUE safe_aci_gatt_update_char_value
static int32_t breath;


/* @brief  Update the value of a characteristic avoiding (for a short time) to
 *         send the next updates if an error in the previous sending has
 *         occurred.
 * @param  servHandle The handle of the service
 * @param  charHandle The handle of the characteristic
 * @param  charValOffset The offset of the characteristic
 * @param  charValueLen The length of the characteristic
 * @param  charValue The pointer to the characteristic
 * @retval tBleStatus Status
 */
tBleStatus safe_aci_gatt_update_char_value(uint16_t servHandle, 
				      uint16_t charHandle,
				      uint8_t charValOffset,
				      uint8_t charValueLen,   
				      const uint8_t *charValue)
{
  tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;
  
  if (breath > 0) {
    breath--;
  } else {
    ret = aci_gatt_update_char_value(servHandle,charHandle,charValOffset,charValueLen,charValue);
    
    if (ret != BLE_STATUS_SUCCESS){
      breath = ACC_BLUENRG_CONGESTION_SKIP;
    }
  }
  
  return (ret);
}

#else /* ACC_BLUENRG_CONGESTION */
#define ACI_GATT_UPDATE_CHAR_VALUE aci_gatt_update_char_value
#endif /* ACC_BLUENRG_CONGESTION */


/**
 * @brief  Add the Config service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ConfigW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  COPY_CONFIG_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3,&ConfigServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS)
    goto fail;

  COPY_CONFIG_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, uuid, 20 /* Max Dimension */,
                           CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &ConfigCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:
  //OSX_BMS_PRINTF("Error while adding Configuration service.\n");
  return BLE_STATUS_ERROR;
}


/**
 * @brief  Add the Console service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ConsoleW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  COPY_CONSOLE_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3*2,&ConsoleW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_TERM_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConsoleW2STHandle, UUID_TYPE_128, uuid, W2ST_CONSOLE_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_WRITE | CHAR_PROP_READ ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &TermCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_STDERR_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConsoleW2STHandle, UUID_TYPE_128, uuid, W2ST_CONSOLE_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &StdErrCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
     goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:
  //OSX_BMS_PRINTF("Error while adding Console service.\n");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update Stderr characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Stderr_Update(uint8_t *data,uint8_t length)
{
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split the code in packages*/
  for(Offset =0; Offset<length; Offset +=W2ST_CONSOLE_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>W2ST_CONSOLE_MAX_CHAR_LEN) ?  W2ST_CONSOLE_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastStderrBuffer,data+Offset,DataToSend);
    LastStderrLen = DataToSend;

    ret = aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS) {
      return BLE_STATUS_ERROR;
    }
    HAL_Delay(10);
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Term_Update(uint8_t *data,uint8_t length)
{
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split the code in packages */
  for(Offset =0; Offset<length; Offset +=W2ST_CONSOLE_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>W2ST_CONSOLE_MAX_CHAR_LEN) ?  W2ST_CONSOLE_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastTermBuffer,data+Offset,DataToSend);
    LastTermLen = DataToSend;

    ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS) {
        OSX_BMS_PRINTF("Error Updating Stdout Char\r\n");
      return BLE_STATUS_ERROR;
    }
    HAL_Delay(20);
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Stderr characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Stderr_Update_AfterRead(void)
{
  tBleStatus ret;

  ret = aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, LastStderrLen , LastStderrBuffer);
  if (ret != BLE_STATUS_SUCCESS) {
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Term_Update_AfterRead(void)
{
  tBleStatus ret;

  ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, LastTermLen , LastTermBuffer);
  if (ret != BLE_STATUS_SUCCESS) {
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Stdout Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating Stdout Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Add the SW Feature service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_SWServW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  uint8_t max_attr_records = 1 /* Sensor Fusion Short */;
#ifdef OSX_BMS_MOTIONAR
  max_attr_records++;
#endif /* OSX_BMS_MOTIONAR */

#ifdef OSX_BMS_MOTIONCP
  max_attr_records++;
#endif /* OSX_BMS_MOTIONCP */

#ifdef OSX_BMS_MOTIONGR
  max_attr_records++;
#endif /* OSX_BMS_MOTIONGR */

#ifdef OSX_BMS_MOTIONPM
  max_attr_records++;
#endif /* OSX_BMS_MOTIONPM */

  COPY_SW_SENS_W2ST_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3*max_attr_records,&SWServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_QUATERNIONS_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(SWServW2STHandle, UUID_TYPE_128, uuid, 2+6*SEND_N_QUATERNIONS,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &QuaternionsCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

#ifdef OSX_BMS_MOTIONAR
  COPY_ACTIVITY_REC_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(SWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &ActivityRecCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
#endif /* OSX_BMS_MOTIONAR */

#ifdef OSX_BMS_MOTIONCP
  COPY_CARRY_POSITION_REC_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(SWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &CarryPosRecCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
#endif /* OSX_BMS_MOTIONCP */
  
#ifdef OSX_BMS_MOTIONGR
  COPY_GESTURE_REC_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(SWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &GestureRecCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
#endif /* OSX_BMS_MOTIONGR */

#ifdef OSX_BMS_MOTIONPM
  COPY_ACC_PEDO_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(SWServW2STHandle, UUID_TYPE_128, uuid, 2+4+2,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AccPedoCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
#endif /* OSX_BMS_MOTIONPM */

  return BLE_STATUS_SUCCESS; 

fail:
  //OSX_BMS_PRINTF("Error while adding SW's Characteristcs service.\n");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update quaternions characteristic value
 * @param  SensorAxes_t *data Structure containing the quaterions
 * @retval tBleStatus      Status
 */
tBleStatus Quat_Update(SensorAxes_t *data)
{
  tBleStatus ret;    

  uint8_t buff[2+ 6*SEND_N_QUATERNIONS];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));

#if SEND_N_QUATERNIONS == 1
  STORE_LE_16(buff+2,data[0].AXIS_X);
  STORE_LE_16(buff+4,data[0].AXIS_Y);
  STORE_LE_16(buff+6,data[0].AXIS_Z);
#elif SEND_N_QUATERNIONS == 2
  STORE_LE_16(buff+2,data[0].AXIS_X);
  STORE_LE_16(buff+4,data[0].AXIS_Y);
  STORE_LE_16(buff+6,data[0].AXIS_Z);

  STORE_LE_16(buff+8 ,data[1].AXIS_X);
  STORE_LE_16(buff+10,data[1].AXIS_Y);
  STORE_LE_16(buff+12,data[1].AXIS_Z);
#elif SEND_N_QUATERNIONS == 3
  STORE_LE_16(buff+2,data[0].AXIS_X);
  STORE_LE_16(buff+4,data[0].AXIS_Y);
  STORE_LE_16(buff+6,data[0].AXIS_Z);

  STORE_LE_16(buff+8 ,data[1].AXIS_X);
  STORE_LE_16(buff+10,data[1].AXIS_Y);
  STORE_LE_16(buff+12,data[1].AXIS_Z);

  STORE_LE_16(buff+14,data[2].AXIS_X);
  STORE_LE_16(buff+16,data[2].AXIS_Y);
  STORE_LE_16(buff+18,data[2].AXIS_Z);
#else
#error SEND_N_QUATERNIONS could be only 1,2,3
#endif
  ret = ACI_GATT_UPDATE_CHAR_VALUE(SWServW2STHandle, QuaternionsCharHandle, 0, 2+6*SEND_N_QUATERNIONS, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Quat Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating Quat Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

#ifdef OSX_BMS_MOTIONAR
/**
 * @brief  Update Activity Recognition value
 * @param  osx_MAR_output_t ActivityCode Activity Recognized
 * @retval tBleStatus      Status
 */
tBleStatus ActivityRec_Update(osx_MAR_output_t ActivityCode)
{
  tBleStatus ret;

  uint8_t buff[2+ 1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = ActivityCode;

  ret = aci_gatt_update_char_value(SWServW2STHandle, ActivityRecCharHandle, 0, 2+1, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating ActivityRec Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating ActivityRec Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
#endif /* OSX_BMS_MOTIONAR */

#ifdef OSX_BMS_MOTIONCP
/**
 * @brief  Update Carry Postion Recognition value
 * @param  osx_MCP_output_t CarryPositionCode Carry Position Recognized
 * @retval tBleStatus      Status
 */
tBleStatus CarryPosRec_Update(osx_MCP_output_t CarryPositionCode)
{
  tBleStatus ret;

  uint8_t buff[2+ 1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = CarryPositionCode;

  ret = aci_gatt_update_char_value(SWServW2STHandle, CarryPosRecCharHandle, 0, 2+1, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating CarryPosRec Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating CarryPosRec Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
#endif /* OSX_BMS_MOTIONCP */


#ifdef OSX_BMS_MOTIONGR
/**
 * @brief  Update Gesture Recognition value
 * @param  osx_MGR_output_t GestureCode Gesture Recognized
 * @retval tBleStatus      Status
 */
tBleStatus GestureRec_Update(osx_MGR_output_t GestureCode)
{
  tBleStatus ret;

  uint8_t buff[2+ 1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = GestureCode;

  ret = aci_gatt_update_char_value(SWServW2STHandle, GestureRecCharHandle, 0, 2+1, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Gesture Rec Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating Gesture Rec Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
#endif /* OSX_BMS_MOTIONGR */

#ifdef OSX_BMS_MOTIONPM
/**
 * @brief  Update Accelerometer Pedometer characteristics
 * @param  osxMPM_output *PM_Data Pedometer result
 * @retval tBleStatus Status
 */
tBleStatus AccPedo_Update(osx_MPM_output_t *PM_Data)
{  
  tBleStatus ret;
  uint8_t buff[2+4+2];
  uint16_t Cadence = (uint16_t) PM_Data->Cadence;

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_LE_32(buff+2,PM_Data->Nsteps);
  STORE_LE_16(buff+6,Cadence);

  ret = aci_gatt_update_char_value(SWServW2STHandle, AccPedoCharHandle, 0, 2+4+2,buff);

  if (ret != BLE_STATUS_SUCCESS){
     if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
       BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating AccPedo Char\r\n");
       Stderr_Update(BufferToWrite,BytesToWrite);
     } else {
       OSX_BMS_PRINTF("Error Updating AccPedo Char\r\n");
     }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
  
}
#endif /* OSX_BMS_MOTIONPM */

/**
 * @brief  Send a notification when the Sensor Fusion Library changes the calibrations status
 * @param  uint32_t Feature Feature calibrated
 * @param  uint8_t Command Replay to this Command
 * @param  uint8_t val Calibration value
 * @retval tBleStatus Status
 */
tBleStatus Calib_Notify(uint32_t Feature,uint8_t Command,uint8_t val)
{
  tBleStatus ret;
  uint8_t buff[2+4+1+1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_BE_32(buff+2,Feature);
  buff[6] = Command;
  buff[7] = (val==0x01) ? 100: val;

  ret = aci_gatt_update_char_value (ConfigServW2STHandle, ConfigCharHandle, 0, 8,buff);
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Configuration Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating Configuration Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/* @brief  Send a notification for answering to a configuration command for Accelerometer events
 * @param  uint32_t Feature Feature calibrated
 * @param  uint8_t Command Replay to this Command
 * @param  uint8_t data result to send back
 * @retval tBleStatus Status
 */
tBleStatus Config_Notify(uint32_t Feature,uint8_t Command,uint8_t data)
{
  tBleStatus ret;
  uint8_t buff[2+4+1+1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_BE_32(buff+2,Feature);
  buff[6] = Command;
  buff[7] = data;

  ret = aci_gatt_update_char_value (ConfigServW2STHandle, ConfigCharHandle, 0, 8,buff);
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Configuration Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating Configuration Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Send a notification When the DS3 detects one Acceleration event
 * @param  Command to Send
 * @retval tBleStatus Status
 */
tBleStatus AccEvent_Notify(uint16_t Command)
{
  tBleStatus ret;
  uint8_t buff[2+2];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_LE_16(buff+2,Command);

  ret = aci_gatt_update_char_value(HWServW2STHandle, AccEventCharHandle, 0, 2+2,buff);
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating AccEvent_Notify Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating AccEvent_Notify Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Add the HW Features service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_HWServW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];
  
  COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE,
                          1+3*4,
                          &HWServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Fill the Environmental BLE Characteristc */
  COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid);
  if(TargetBoardFeatures.NumTempSensors==2) {
    uuid[14] |= 0x05; /* Two Temperature values*/
    EnvironmentalCharSize+=2*2;
  } else if(TargetBoardFeatures.NumTempSensors==1) {
    uuid[14] |= 0x04; /* One Temperature value*/
    EnvironmentalCharSize+=2;
  }

  if(TargetBoardFeatures.HandleHumSensor) {
   uuid[14] |= 0x08; /* Humidity */
   EnvironmentalCharSize+=2;
  }

  if(TargetBoardFeatures.HandlePressSensor) {
    uuid[14] |= 0x10; /* Pressure value*/
    EnvironmentalCharSize+=4;
  }

  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, EnvironmentalCharSize,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &EnvironmentalCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+3*3*2,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AccGyroMagCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_ACC_EVENT_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+2,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AccEventCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_LED_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &LedCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:
  //OSX_BMS_PRINTF("Error while adding HW's Characteristcs service.\n");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update acceleration/Gryoscope and Magneto characteristics value
 * @param  SensorAxes_t Acc Structure containing acceleration value in mg
 * @param  SensorAxes_t Gyro Structure containing Gyroscope value
 * @param  SensorAxes_t Mag Structure containing magneto value
 * @retval tBleStatus      Status
 */
tBleStatus AccGyroMag_Update(SensorAxes_t *Acc,SensorAxes_t *Gyro,SensorAxes_t *Mag)
{  
  tBleStatus ret;
  int32_t AXIS_X;
  int32_t AXIS_Y;
  int32_t AXIS_Z;

  uint8_t buff[2+3*3*2];

  STORE_LE_16(buff   ,(HAL_GetTick()>>3));
  
  STORE_LE_16(buff+2 ,Acc->AXIS_X);
  STORE_LE_16(buff+4 ,Acc->AXIS_Y);
  STORE_LE_16(buff+6 ,Acc->AXIS_Z);
  
  Gyro->AXIS_X/=100;
  Gyro->AXIS_Y/=100;
  Gyro->AXIS_Z/=100;

  STORE_LE_16(buff+8 ,Gyro->AXIS_X);
  STORE_LE_16(buff+10,Gyro->AXIS_Y);
  STORE_LE_16(buff+12,Gyro->AXIS_Z);
    
  /* Apply Magneto calibration */
  AXIS_X = Mag->AXIS_X - magOffset.magOffX;
  AXIS_Y = Mag->AXIS_Y - magOffset.magOffY;
  AXIS_Z = Mag->AXIS_Z - magOffset.magOffZ;

  STORE_LE_16(buff+14,AXIS_X);
  STORE_LE_16(buff+16,AXIS_Y);
  STORE_LE_16(buff+18,AXIS_Z);
  
  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServW2STHandle, AccGyroMagCharHandle, 0, 2+3*3*2, buff);
	
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Acc/Gyro/Mag Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating Acc/Gyro/Mag Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;	
}

/**
 * @brief  Update Environmental characteristic value
 * @param  int32_t Press Pressure in mbar
 * @param  uint16_t Hum humidity RH (Relative Humidity) in thenths of %
 * @param  int16_t Temp2 Temperature in tenths of degree second sensor
 * @param  int16_t Temp1 Temperature in tenths of degree first sensor
 * @retval tBleStatus   Status
 */
tBleStatus Environmental_Update(int32_t Press,uint16_t Hum,int16_t Temp2,int16_t Temp1)
{
  tBleStatus ret;
  uint8_t BuffPos;
  
  uint8_t buff[2+4/*Press*/+2/*Hum*/+2/*Temp2*/+2/*Temp1*/];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  BuffPos=2;

  if(TargetBoardFeatures.HandlePressSensor) {
    STORE_LE_32(buff+BuffPos,Press);
    BuffPos+=4;
  }

  if(TargetBoardFeatures.HandleHumSensor) {
    STORE_LE_16(buff+BuffPos,Hum);
    BuffPos+=2;
  }

  if(TargetBoardFeatures.NumTempSensors==2) {
    STORE_LE_16(buff+BuffPos,Temp2);
    BuffPos+=2;
    
    STORE_LE_16(buff+BuffPos,Temp1);
    BuffPos+=2;
  } else if(TargetBoardFeatures.NumTempSensors==1) {
    STORE_LE_16(buff+BuffPos,Temp1);
    BuffPos+=2;
  }

  ret = aci_gatt_update_char_value(HWServW2STHandle, EnvironmentalCharHandle, 0, EnvironmentalCharSize,buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Environmental Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating Environmental Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update LEDs characteristic value
 * @param  uint8_t LedStatus LEDs status 0/1 (off/on)
 * @retval tBleStatus   Status
 */
tBleStatus LED_Update(uint8_t LedStatus)
{
  tBleStatus ret;

  uint8_t buff[2+1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = LedStatus;

  ret = aci_gatt_update_char_value(HWServW2STHandle, LedCharHandle, 0, 2+1,buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating LED Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating Temp Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Puts the device in connectable mode.
 * @param  None 
 * @retval None
 */
void setConnectable(void)
{  
  char local_name[8] = {AD_TYPE_COMPLETE_LOCAL_NAME,NAME_BLUEMS};
  uint8_t manuf_data[20] = {
    2,0x0A,0x00 /* 0 dBm */, // Trasmission Power
    8,0x09,NAME_BLUEMS, // Complete Name
    7,0xFF,0x01/*SKD version */,
#ifdef  STM32_NUCLEO
           0x80,
#elif STM32_SENSORTILE
           0x02,
#endif /* STM32_NUCLEO */
           0x00 /* */,
           0xE0 /* ACC+Gyro+Mag*/,
           0x00 /*  */,
           0x00 /*  */
  };

  manuf_data[16] |= 0x20; /* Led */

  if(TargetBoardFeatures.NumTempSensors==2) {
    manuf_data[17] |= 0x05; /* Two Temperature values*/
  } else if(TargetBoardFeatures.NumTempSensors==1) {
    manuf_data[17] |= 0x04; /* One Temperature value*/
  }

  if(TargetBoardFeatures.HandleHumSensor) {
    manuf_data[17] |= 0x08; /* Humidity */
  }

  if(TargetBoardFeatures.HandlePressSensor) {
    manuf_data[17] |= 0x10; /* Pressure value*/
  }
  
  /* DS3 DIL24  present*/
  if(TargetBoardFeatures.HWAdvanceFeatures) {    
    /* Accelerometer Events */
    manuf_data[18] |=0x04;    
  }
  
  /* Sensor Fusion */
  if(TargetBoardFeatures.osxMotionFXIsInitalized) {
    manuf_data[18] |= 0x01;
  }

  /* ActivityRec */
#ifdef OSX_BMS_MOTIONAR
  if(TargetBoardFeatures.osxMotionARIsInitalized) {
    manuf_data[19] |= 0x10;
  }
#endif /* OSX_BMS_MOTIONAR */

  /* CarryPosRec */
#ifdef OSX_BMS_MOTIONCP
  if(TargetBoardFeatures.osxMotionCPIsInitalized) {
    manuf_data[19] |= 0x08;
  }
#endif /* OSX_BMS_MOTIONCP */

  /* Gesture Rec */
#ifdef OSX_BMS_MOTIONGR
  if(TargetBoardFeatures.osxMotionGRIsInitalized) {
    manuf_data[19] |= 0x02;
  }
#endif /* OSX_BMS_MOTIONGR */

  /* Pedometer */
#ifdef OSX_BMS_MOTIONPM
  if(TargetBoardFeatures.osxMotionPMIsInitalized) {
    manuf_data[19] |= 0x01;
  }
#endif /* OSX_BMS_MOTIONPM */  

  /* disable scan response */
  hci_le_set_scan_resp_data(0,NULL);
  aci_gap_set_discoverable(ADV_IND, 0, 0,
#ifndef MAC_BLUEMS
                           STATIC_RANDOM_ADDR,
#else /* MAC_BLUEMS */
                           PUBLIC_ADDR,
#endif /* MAC_BLUEMS */
                           NO_WHITE_LIST_USE,
                           sizeof(local_name), local_name, 0, NULL, 0, 0);

  /* Send Advertising data */
  aci_gap_update_adv_data(20, manuf_data);
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t addr[6] Address of peer device
 * @param  uint16_t handle Connection handle
 * @retval None
 */
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{  
  connected = TRUE;
  connection_handle = handle;

#ifdef OSX_BMS_DEBUG_CONNECTION
  OSX_BMS_PRINTF(">>>>>>CONNECTED %x:%x:%x:%x:%x:%x\r\n",addr[5],addr[4],addr[3],addr[2],addr[1],addr[0]);
#endif /* OSX_BMS_DEBUG_CONNECTION */

  ConnectionBleStatus=0;
  
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    DisableHWFeatures();
  }

  ForceReCalibration     =0;
  FirstConnectionConfig  =0;
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None 
 * @retval None
 */
static void GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;

#ifdef OSX_BMS_DEBUG_CONNECTION  
  OSX_BMS_PRINTF("<<<<<<DISCONNECTED\r\n");
#endif /* OSX_BMS_DEBUG_CONNECTION */  

  /* Make the device connectable again. */
  set_connectable = TRUE;

  ConnectionBleStatus=0;
  
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    DisableHWFeatures();
  }

  ForceReCalibration     =0;
  FirstConnectionConfig  =0;
}

/**
 * @brief  This function is called when there is a Bluetooth Read request
 * @param  uint16_t handle Handle of the attribute
 * @retval None
 */
void Read_Request_CB(uint16_t handle)
{
  uint8_t Status;  
  if(handle == EnvironmentalCharHandle + 1){
    /* Read Request for Pressure,Humidity, and Temperatures*/
    float SensorValue;
    int32_t PressToSend=0;
    uint16_t HumToSend=0;
    int16_t Temp2ToSend=0,Temp1ToSend=0;
    int32_t decPart, intPart;

    if(TargetBoardFeatures.HandlePressSensor) {
      if(BSP_PRESSURE_IsInitialized(TargetBoardFeatures.HandlePressSensor,&Status)==COMPONENT_OK) {
        BSP_PRESSURE_Get_Press(TargetBoardFeatures.HandlePressSensor,(float *)&SensorValue);
        MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
        PressToSend=intPart*100+decPart;
      }
    }

    if(TargetBoardFeatures.HandleHumSensor) {
      if(BSP_HUMIDITY_IsInitialized(TargetBoardFeatures.HandleHumSensor,&Status)==COMPONENT_OK){
        BSP_HUMIDITY_Get_Hum(TargetBoardFeatures.HandleHumSensor,(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        HumToSend = intPart*10+decPart;
      }
    }

    if(TargetBoardFeatures.NumTempSensors==2) {
      if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK){
        BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp1ToSend = intPart*10+decPart; 
      }

      if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[1],&Status)==COMPONENT_OK){
        BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[1],(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp2ToSend = intPart*10+decPart; 
      }
    } else if(TargetBoardFeatures.NumTempSensors==1) {
      if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK){
        BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp1ToSend = intPart*10+decPart; 
      }
    }
    Environmental_Update(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
  } else if(handle == LedCharHandle + 1){
    /* Read Request for Led Status */
    LED_Update(TargetBoardFeatures.LedStatus);
  } else if(handle == AccEventCharHandle +1) {
    /* Read Request for Acc Pedometer DS3 */
    if(TargetBoardFeatures.HWAdvanceFeatures){
      uint16_t StepCount;
      if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
        StepCount = GetStepHWPedometer();
      } else {
        StepCount = 0;
      }
      AccEvent_Notify(StepCount);
    }
#ifdef OSX_BMS_MOTIONPM
   } else if(handle == AccPedoCharHandle + 1){
    /* Read Request for osxMotionPM */
    AccPedo_Update(&PM_DataOUT);
#endif /* OSX_BMS_MOTIONPM */
  }else if (handle == StdErrCharHandle + 1) {
    /* Send again the last packet for StdError */
    Stderr_Update_AfterRead();
  } else if (handle == TermCharHandle + 1) {
    /* Send again the last packet for Terminal */
    Term_Update_AfterRead();
#ifdef OSX_BMS_MOTIONAR
  }else if(handle == ActivityRecCharHandle + 1){
     ActivityRec_Update(ActivityCode);
#endif /* OSX_BMS_MOTIONAR */
#ifdef OSX_BMS_MOTIONCP
  }else if(handle == CarryPosRecCharHandle + 1){
    CarryPosRec_Update(CarryPositionCode);
#endif /* OSX_BMS_MOTIONCP */
#ifdef OSX_BMS_MOTIONGR
  }else if(handle == GestureRecCharHandle + 1){
    GestureRec_Update(GestureRecognitionCode);
#endif /* OSX_BMS_MOTIONGR */    
  }

  //EXIT:
  if(connection_handle != 0)
    aci_gatt_allow_read(connection_handle);
}

/**
 * @brief  This function is called when there is a change on the gatt attribute
 * With this function it's possible to understand if one application 
 * is subscribed or not to the one service
 * @param uint16_t att_handle Handle of the attribute
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval None
 */
void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length) {
   if(attr_handle == QuaternionsCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_QUAT);

      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }
      /* Set the new Capture compare value */
      {
        uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
        /* Set the Capture Compare Register value */
        __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));
      }
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_QUAT);

      /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }
    }
#ifdef OSX_BMS_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->Quater=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_QUAT) ? "ON" : "OFF");
     Term_Update(BufferToWrite,BytesToWrite);
    } else
      OSX_BMS_PRINTF("--->Quater=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_QUAT) ? "ON" : "OFF");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  }
#ifdef OSX_BMS_MOTIONAR
   else if(attr_handle == ActivityRecCharHandle + 2){
     if (att_data[0] == 01) {

       Set4GAccelerometerFullScale();

       W2ST_ON_CONNECTION(W2ST_CONNECT_AR);

      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }
      /* Set the new Capture compare value */
      {
        uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
        /* Set the Capture Compare Register value */
        __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_3, (uhCapture + uhCCR3_Val));
      }
     } else if (att_data[0] == 0){

       Set2GAccelerometerFullScale();

       W2ST_OFF_CONNECTION(W2ST_CONNECT_AR);

      /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }
     }
#ifdef OSX_BMS_DEBUG_CONNECTION
     if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite =sprintf((char *)BufferToWrite,"--->ActRec=%s\r\n",  W2ST_CHECK_CONNECTION(W2ST_CONNECT_AR) ? "ON" : "OFF");
       Term_Update(BufferToWrite,BytesToWrite);
     } else
       OSX_BMS_PRINTF("--->ActRec=%s\r\n",  W2ST_CHECK_CONNECTION(W2ST_CONNECT_AR) ? "ON" : "OFF");
#endif /* OSX_BMS_DEBUG_CONNECTION */
   }
#endif /* OSX_BMS_MOTIONAR */
#ifdef OSX_BMS_MOTIONCP
   else if(attr_handle == CarryPosRecCharHandle + 2){
     if (att_data[0] == 01) {

      Set4GAccelerometerFullScale();

       W2ST_ON_CONNECTION(W2ST_CONNECT_CP);

      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }
      /* Set the new Capture compare value */
      {
        uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
        /* Set the Capture Compare Register value */
        __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + uhCCR2_Val));
      }
     } else if (att_data[0] == 0){

       Set2GAccelerometerFullScale();

       W2ST_OFF_CONNECTION(W2ST_CONNECT_CP);

       /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }
     }
#ifdef OSX_BMS_DEBUG_CONNECTION
     if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite =sprintf((char *)BufferToWrite,"--->CarryPosRec=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_CP) ? "ON" : "OFF");
       Term_Update(BufferToWrite,BytesToWrite);
     } else
       OSX_BMS_PRINTF("--->CarryPosRec=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_CP) ? "ON" : "OFF");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  }
#endif /* OSX_BMS_MOTIONCP */
#ifdef OSX_BMS_MOTIONGR
   else if(attr_handle == GestureRecCharHandle + 2){
     if (att_data[0] == 01) {

       Set4GAccelerometerFullScale();

       W2ST_ON_CONNECTION(W2ST_CONNECT_GR);

      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }
      /* Set the new Capture compare value */
      {
        uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
        /* Set the Capture Compare Register value */
        __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));
      }
     } else if (att_data[0] == 0){

       Set2GAccelerometerFullScale();

       W2ST_OFF_CONNECTION(W2ST_CONNECT_GR);

       /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }
     }
#ifdef OSX_BMS_DEBUG_CONNECTION
     if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite =sprintf((char *)BufferToWrite,"--->GestureRec=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_GR) ? "ON" : "OFF");
       Term_Update(BufferToWrite,BytesToWrite);
     } else
       OSX_BMS_PRINTF("--->GestureRec=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_GR) ? "ON" : "OFF");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  }
#endif /* OSX_BMS_MOTIONGR */
#ifdef OSX_BMS_MOTIONPM
   else if(attr_handle == AccPedoCharHandle + 2){
     if (att_data[0] == 01) {

       Set4GAccelerometerFullScale();

       W2ST_ON_CONNECTION(W2ST_CONNECT_PM);

      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }
      /* Set the new Capture compare value */
      {
        uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
        /* Set the Capture Compare Register value */
        __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + uhCCR2_Val));
      }
     } else if (att_data[0] == 0){

       Set2GAccelerometerFullScale();

       W2ST_OFF_CONNECTION(W2ST_CONNECT_PM);

       /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }
     }
#ifdef OSX_BMS_DEBUG_CONNECTION
     if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite =sprintf((char *)BufferToWrite,"--->AccPedo=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_PM) ? "ON" : "OFF");
       Term_Update(BufferToWrite,BytesToWrite);
     } else
       OSX_BMS_PRINTF("--->AccPedo=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_PM) ? "ON" : "OFF");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  }
#endif /* OSX_BMS_MOTIONPM */
  else if(attr_handle == ConfigCharHandle + 2){
    if (att_data[0] == 01) {
      FirstConnectionConfig=1;
    } else if (att_data[0] == 0){
      FirstConnectionConfig=0;
    }
#ifdef OSX_BMS_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->Calib=%s\r\n", FirstConnectionConfig ? "ON" : "OFF");
     Term_Update(BufferToWrite,BytesToWrite);
    } else
      OSX_BMS_PRINTF("--->Calib=%s\r\n", FirstConnectionConfig ? "ON" : "OFF");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  } else if(attr_handle == AccGyroMagCharHandle + 2) {
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG);

      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }

      /* Set the new Capture compare value */
      {
        uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
        /* Set the Capture Compare Register value */
        __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));
      }
    } else if (att_data[0] == 0) {
      W2ST_OFF_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG);

      /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }      
    }
#ifdef OSX_BMS_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->Acc/Gyro/Mag=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG) ? "ON" : "OFF");
      Term_Update(BufferToWrite,BytesToWrite);
    } else
      OSX_BMS_PRINTF("--->Acc/Gyro/Mag=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG) ? "ON" : "OFF");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  } else if(attr_handle == AccEventCharHandle + 2) {
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_ACC_EVENT);
    } else if (att_data[0] == 0) {
      W2ST_OFF_CONNECTION(W2ST_CONNECT_ACC_EVENT);
      DisableHWFeatures();
    }
#ifdef OSX_BMS_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->AccEvent=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_EVENT) ? "ON" : "OFF");
     Term_Update(BufferToWrite,BytesToWrite);
    } else
      OSX_BMS_PRINTF("--->AccEvent=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_EVENT) ? "ON" : "OFF");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  } else if(attr_handle == EnvironmentalCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_ENV);

      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_Base_Start_IT(&TimEnvHandle) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_ENV);

      /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_Base_Stop_IT(&TimEnvHandle) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }
    }
#ifdef OSX_BMS_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->Env=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV) ? "ON" : "OFF");
     Term_Update(BufferToWrite,BytesToWrite);
    } else
      OSX_BMS_PRINTF("--->Env=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV) ? "ON" : "OFF");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  } else if(attr_handle == StdErrCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_STD_ERR);
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_ERR);
    }
  } else if(attr_handle == TermCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_STD_TERM);
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_TERM);
    }
  } else if (attr_handle == TermCharHandle + 1){
    uint32_t SendBackData =1; /* By default Answer with the same message received */

    if(SizeOfUpdateBlueFW!=0) {
      /* BLUEMICROSYSTEM1 firwmare update */
      int8_t RetValue = UpdateFWBlueMS(&SizeOfUpdateBlueFW,att_data, data_length,1);
      if(RetValue!=0) {
        MCR_FAST_TERM_UPDATE_FOR_OTA(((uint8_t *)&RetValue));
        if(RetValue==1) {
          /* if OTA checked */
          BytesToWrite =sprintf((char *)BufferToWrite,"The Board will restart in 5 seconds\r\n");
          Term_Update(BufferToWrite,BytesToWrite);
          OSX_BMS_PRINTF("BLUEMICROSYSTEM1 will restart in 5 seconds\r\n");
          HAL_Delay(5000);
          HAL_NVIC_SystemReset();
        }
      }
      SendBackData=0;
    } else {
      /* Received one write from Client on Terminal characteristc */
      SendBackData = DebugConsoleCommandParsing(att_data,data_length);
    }

    /* Send it back for testing */
    if(SendBackData) {
			uint8_t value[20];
			memcpy(value, (uint8_t*)att_data, data_length);
			value[data_length]='1';
      Term_Update(value,data_length+1);
			/*
			uint8_t ch =data_length+48;
			uint8_t *str= &ch;
				int8_t send_temp[18]={0};
			int Num=1;
			int16_t p16Data[Num];	
			for (int i = 0; i < Num; i++) {
				p16Data[i] = (((int16_t)*(att_data +i* 2 + 1)) << 8) + (int16_t)*(att_data+i*2+2);
			}
	//		p16Data[0]*=2;
			send_temp[0]='s';
				for(int i=0;i<2*Num;i++){
       send_temp[i+1]=(int8_t)(p16Data[i/2]>>(8-(i%2)*8));
				send_temp[1+2*Num]+=send_temp[i+1];
       }
			send_temp[1+2*Num]+=send_temp[0];
		//	 memcpy(RXMessage.value, (uint8_t*)send_temp, 4);
		//	Term_Update((uint8_t*)send_temp,18);
			 Term_Update(att_data,data_length);
			 */
    }
  } else if(attr_handle == LedCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_LED);
      /* Update the LED feature */
      LED_Update(TargetBoardFeatures.LedStatus);
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_LED);
    }
#ifdef OSX_BMS_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->Led=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_LED) ? "ON" : "OFF");
     Term_Update(BufferToWrite,BytesToWrite);
    } else
      OSX_BMS_PRINTF("--->Led=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_LED) ? "ON" : "OFF");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  } else if (attr_handle == ConfigCharHandle + 1) {
    /* Received one write command from Client on Configuration characteristc */
    ConfigCommandParsing(att_data, data_length);    
  } else {
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Notification UNKNOW handle\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Notification UNKNOW handle\r\n");
    }
  }
}

/**
 * @brief  This function makes the parsing of the Debug Console Commands
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval uint32_t SendItBack true/false
 */
static uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length)
{
  uint32_t SendBackData = 1;
  static uint32_t osxMotionLicStart=0;

  if(osxMotionLicStart==0){
    /* No osxMotion License activation phase */
    if((att_data[0]=='?') & (att_data[1]=='?')) {
      /* Print Legend */
      SendBackData=0;

      BytesToWrite =sprintf((char *)BufferToWrite,"Command:\r\n"
        "pr->HW pedometer reset\r\n"
#ifndef STM32_NUCLEO
        "fw->Jump to Boot Loader\r\n"
#endif /* STM32_NUCLEO */
         "info-> System Info\r\n"
         "versionFw-> FW Version\r\n"
         "versionBle-> Ble Version\r\n");
      Term_Update(BufferToWrite,BytesToWrite);
    } else if((att_data[0]=='p') & (att_data[1]=='r')) {
    /* Reset the pedometer DS3 HW counter */
      ResetHWPedometer();
      SendBackData=0;
    }
#ifndef STM32_NUCLEO
    else if((att_data[0]=='f') & (att_data[1]=='w')) {
      /* Jump to STM32 System Memory for firmware upgrade - Bootloader */
      BytesToWrite =sprintf((char *)BufferToWrite,"\r\nThe board restarts\r\n");
      Term_Update(BufferToWrite,BytesToWrite);
      BytesToWrite =sprintf((char *)BufferToWrite,"in Boot Loader mode\r\n");
      Term_Update(BufferToWrite,BytesToWrite);      
  #if defined (__IAR_SYSTEMS_ICC__)
    RestartInBootLoaderMode = BLUEMSYS_CHECK_JUMP_TO_BOOTLOADER;
  #elif defined (__CC_ARM)
    *RestartInBootLoaderMode = BLUEMSYS_CHECK_JUMP_TO_BOOTLOADER;
  #elif defined (__GNUC__)
    RestartInBootLoaderMode = BLUEMSYS_CHECK_JUMP_TO_BOOTLOADER;
  #else
    #error "Toolchain not supported"
  #endif
      HAL_NVIC_SystemReset();
    }
#endif /* STM32_NUCLEO */
    else if(!strncmp("versionFw",(char *)(att_data),9)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"%s_%s_%c.%c.%c\r\n",
#ifdef STM32F401xE
                            "F401"
#elif STM32F446xx
                            "F446"
#elif STM32L476xx
                            "L476"
#else
#error "Undefined STM32 processor type"
#endif
                            ,OSX_BMS_PACKAGENAME,
                            OSX_BMS_VERSION_MAJOR,
                            OSX_BMS_VERSION_MINOR,
                            OSX_BMS_VERSION_PATCH);
      Term_Update(BufferToWrite,BytesToWrite);
      SendBackData=0;
    }  else if(!strncmp("info",(char *)(att_data),4)) {
      SendBackData=0;
      
      BytesToWrite =sprintf((char *)BufferToWrite,"\r\nSTMicroelectronics %s:\r\n"
         "\tVersion %c.%c.%c\r\n"
#ifdef USE_STM32F4XX_NUCLEO
        "\tSTM32F401RE-Nucleo board"
#elif USE_STM32L4XX_NUCLEO
  #ifdef STM32_SENSORTILE
        "\tSTM32476RG-SensorTile board"
  #elif STM32_NUCLEO
        "\tSTM32L476RG-Nucleo board"
  #endif /* STM32_SENSORTILE */
#endif /* USE_STM32L4XX_NUCLEO */
          "\r\n",
          OSX_BMS_PACKAGENAME,
          OSX_BMS_VERSION_MAJOR,OSX_BMS_VERSION_MINOR,OSX_BMS_VERSION_PATCH);
      Term_Update(BufferToWrite,BytesToWrite);

      BytesToWrite =sprintf((char *)BufferToWrite,"\t(HAL %ld.%ld.%ld_%ld)\r\n"
        "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n",
#elif defined (__CC_ARM)
        " (KEIL)\r\n",
#elif defined (__GNUC__)
        " (openstm32)\r\n",
#endif
          HAL_GetHalVersion() >>24,
          (HAL_GetHalVersion() >>16)&0xFF,
          (HAL_GetHalVersion() >> 8)&0xFF,
           HAL_GetHalVersion()      &0xFF,
           __DATE__,__TIME__);
      Term_Update(BufferToWrite,BytesToWrite);

    } else if(!strncmp("upgradeFw",(char *)(att_data),9)) {
      uint32_t uwCRCValue;
      uint8_t *PointerByte = (uint8_t*) &SizeOfUpdateBlueFW;

      SizeOfUpdateBlueFW=atoi((char *)(att_data+9));
      PointerByte[0]=att_data[ 9];
      PointerByte[1]=att_data[10];
      PointerByte[2]=att_data[11];
      PointerByte[3]=att_data[12];
      PointerByte = (uint8_t*) &uwCRCValue;
      PointerByte[0]=att_data[13];
      PointerByte[1]=att_data[14];
      PointerByte[2]=att_data[15];
      PointerByte[3]=att_data[16];

      OSX_BMS_PRINTF("OTA BLUEMICROSYSTEM1 SIZE=%ld uwCRCValue=%lx\r\n",SizeOfUpdateBlueFW,uwCRCValue);

      /* Reset the Flash */
      StartUpdateFWBlueMS(SizeOfUpdateBlueFW,uwCRCValue);

#ifdef USE_STM32F4XX_NUCLEO
      /* Save the osxLicenses if's necessary and only for F401RE */
      if(osxLicencesManager.osxLicenseManagerIntialized){
        SaveLicensesStatus();
        NecessityToSaveLicense=0;
     }
#endif /* USE_STM32F4XX_NUCLEO */

      /* Reduce the connection interval */
      {
        int ret = aci_l2cap_connection_parameter_update_request(connection_handle,
                                                      10 /* interval_min*/,
                                                      10 /* interval_max */,
                                                      0   /* slave_latency */,
                                                      400 /*timeout_multiplier*/);
        /* Go to infinite loop if there is one error */
        if (ret != BLE_STATUS_SUCCESS) {
          while (1) {
            OSX_BMS_PRINTF("Problem Changing the connection interval\r\n");
          }
        }
      }

      /* Signal that we are ready sending back the CRC value*/
      BufferToWrite[0] = PointerByte[0];
      BufferToWrite[1] = PointerByte[1];
      BufferToWrite[2] = PointerByte[2];
      BufferToWrite[3] = PointerByte[3];
      BytesToWrite = 4;
      Term_Update(BufferToWrite,BytesToWrite);

      SendBackData=0;
    } else if(!strncmp("versionBle",(char *)(att_data),10)) {
      uint8_t  hwVersion;
      uint16_t fwVersion;
      /* get the BlueNRG HW and FW versions */
      getBlueNRGVersion(&hwVersion, &fwVersion);
      BytesToWrite =sprintf((char *)BufferToWrite,"%s_%d.%d.%c\r\n",
                            (hwVersion > 0x30) ? "BleMS" : "Ble",
                            fwVersion>>8, 
                            (fwVersion>>4)&0xF,
                            (hwVersion > 0x30) ? ('a'+(fwVersion&0xF)-1) : 'a');
      Term_Update(BufferToWrite,BytesToWrite);
      SendBackData=0;
    } else if((att_data[0]=='u') & (att_data[1]=='i') & (att_data[2]=='d')) {
      /* Write back the STM32 UID */
      uint8_t *uid = (uint8_t *)STM32_UUID;
      uint32_t MCU_ID = STM32_MCU_ID[0]&0xFFF;
      BytesToWrite =sprintf((char *)BufferToWrite,"%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X_%.3lX\r\n",
                            uid[ 3],uid[ 2],uid[ 1],uid[ 0],
                            uid[ 7],uid[ 6],uid[ 5],uid[ 4],
                            uid[11],uid[ 10],uid[9],uid[8],
                            MCU_ID);
      Term_Update(BufferToWrite,BytesToWrite);
      SendBackData=0;
    } else if((att_data[0]=='l') & (att_data[1]=='i') & (att_data[2]=='c')) {
      /* Write back the osxMotion License status */
      LicensesStatus();
      SendBackData=0;
    } else if((att_data[0]=='X') & (att_data[1]=='X') & (att_data[2]=='1')){
      SendBackData=0;
      BytesToWrite =sprintf((char *)BufferToWrite,"Activate of All the osxMotion licenses\r\n");
      Term_Update(BufferToWrite,BytesToWrite);
      ActivateAllLicenses();
    } else if((att_data[0]=='X') & (att_data[1]=='X') & (att_data[2]=='0')){
      ResetLicensesStatus();
      BytesToWrite =sprintf((char *)BufferToWrite,"Reset of All the osxMotion licenses\r\n");
      Term_Update(BufferToWrite,BytesToWrite);
      BytesToWrite =sprintf((char *)BufferToWrite,"The Board will restart in 5 seconds\r\n");
      Term_Update(BufferToWrite,BytesToWrite);
      HAL_Delay(5000);
      HAL_NVIC_SystemReset();
      SendBackData=0;
    } else {
      /* Try to understand if we want to Initializes one osxMotion License */
      osxMotionLicStart = LicenseParsingConsoleCommand(att_data, data_length);
      if(osxMotionLicStart) {
        SendBackData=0;
      }
    }
  } else {
    /* We need to Finalize one osxMotion License */
    osxMotionLicStart = LicenseParsingConsoleCommand(att_data, data_length);
    SendBackData=0;
  }
  return SendBackData;
}

/**
 * @brief  This function makes the parsing of the Configuration Commands
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval None
 */
static void ConfigCommandParsing(uint8_t * att_data, uint8_t data_length)
{
  uint32_t FeatureMask = (att_data[3]) | (att_data[2]<<8) | (att_data[1]<<16) | (att_data[0]<<24);
  uint8_t Command = att_data[4];
  uint8_t Data    = att_data[5];

  switch (FeatureMask) {
    case FEATURE_MASK_SENSORFUSION_SHORT:
      /* Sensor Fusion */
      switch (Command) {
        case W2ST_COMMAND_CAL_STATUS:
#ifdef OSX_BMS_DEBUG_CONNECTION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite =sprintf((char *)BufferToWrite,"Calibration STATUS Signal For Features=%lx\n\r",FeatureMask);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            OSX_BMS_PRINTF("Calibration STATUS Signal For Features=%lx\n\r",FeatureMask);
          }
#endif /* OSX_BMS_DEBUG_CONNECTION */
          /* Replay with the calibration status for the feature */
          /* Control the calibration status */
          {
            unsigned char isCal = osx_MotionFX_compass_isCalibrated();
            Calib_Notify(FeatureMask,Command,isCal);
          }
        break;
        case W2ST_COMMAND_CAL_RESET:
#ifdef OSX_BMS_DEBUG_CONNECTION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite =sprintf((char *)BufferToWrite,"Calibration RESET Signal For Feature=%lx\n\r",FeatureMask);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            OSX_BMS_PRINTF("Calibration RESET Signal For Feature=%lx\n\r",FeatureMask);
          }
#endif /* OSX_BMS_DEBUG_CONNECTION */
          /* Reset the calibration */
          ForceReCalibration=1;
        break;
        case W2ST_COMMAND_CAL_STOP:
#ifdef OSX_BMS_DEBUG_CONNECTION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite =sprintf((char *)BufferToWrite,"Calibration STOP Signal For Feature=%lx\n\r",FeatureMask);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            OSX_BMS_PRINTF("Calibration STOP Signal For Feature=%lx\n\r",FeatureMask);
          }
#endif /* OSX_BMS_DEBUG_CONNECTION */
          /* Do nothing in this case */
        break;
        default:
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
            BytesToWrite =sprintf((char *)BufferToWrite, "Calibration UNKNOW Signal For Feature=%lx\n\r",FeatureMask);
            Stderr_Update(BufferToWrite,BytesToWrite);
          } else {
            OSX_BMS_PRINTF("Calibration UNKNOW Signal For Feature=%lx\n\r",FeatureMask);
          }
      }      
    break;
  case FEATURE_MASK_ACC_EVENTS:
    /* Acc events */
#ifdef OSX_BMS_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"Conf Sig F=%lx C=%c D=%x\n\r",FeatureMask,Command,Data);
      Term_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Conf Sig F=%lx C=%c D=%x\n\r",FeatureMask,Command,Data);
    }
#endif /* OSX_BMS_DEBUG_CONNECTION */      
    switch(Command) {
      case 'f':
        /* FreeFall */
        switch(Data) {
          case 1:
            EnableHWFreeFall();
             Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWFreeFall();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         }
      break;
      case 'd':
        /* Double Tap */
        switch(Data) {
          case 1:
            EnableHWDoubleTap();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWDoubleTap();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
      break;
      case 's':
        /* Single Tap */
        switch(Data) {
          case 1:
            EnableHWSingleTap();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWSingleTap();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
      break;
      case 'p':
        /* Pedometer */
        switch(Data) {
          case 1:
            EnableHWPedometer();
            ResetHWPedometer();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWPedometer();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
       break;
      case 'w':
        /* Wake UP */
        switch(Data) {
          case 1:
            EnableHWWakeUp();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWWakeUp();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
       break;
       case 't':
         /* Tilt */
        switch(Data) {
          case 1:
            EnableHWTilt();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWTilt();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
      break;
      case 'o' :
        /* Tilt */
        switch(Data) {
        case 1:
          EnableHWOrientation6D();
          Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
          break;
        case 0:
          DisableHWOrientation6D();
          Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
          break;
        }
      break;
    }
    break;
    case FEATURE_MASK_LED:
      /* Led events */
#ifdef OSX_BMS_DEBUG_CONNECTION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
        BytesToWrite = sprintf((char *)BufferToWrite,"Conf Sig F=%lx C=%c D=%x\n\r",FeatureMask,Command,Data);
        Term_Update(BufferToWrite,BytesToWrite);
      } else {
        OSX_BMS_PRINTF("Conf Sig F=%lx C=%c D=%x\n\r",FeatureMask,Command,Data);
      }
#endif /* OSX_BMS_DEBUG_CONNECTION */
     switch(Command) {
      case 1:
        TargetBoardFeatures.LedStatus=1;
        LedOnTargetPlatform();
        Config_Notify(FEATURE_MASK_LED,Command,Data);
        break;
      case 0:
        TargetBoardFeatures.LedStatus=0;
        LedOffTargetPlatform();
        Config_Notify(FEATURE_MASK_LED,Command,Data);
        break;
     }
     /* Update the LED feature */
     if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_LED)) {
       LED_Update(TargetBoardFeatures.LedStatus);
     }
    break;
  }
}

/**
 * @brief  This function is called whenever there is an ACI event to be processed.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void *pckt Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = pckt;
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
  if(hci_pckt->type != HCI_EVENT_PKT) {
    return;
  }
  
  switch(event_pckt->evt){
    
  case EVT_DISCONN_COMPLETE:
    {
      GAP_DisconnectionComplete_CB();
    }
    break;
  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;
      
      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (void *)evt->data;
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
      }
    }
    break;
  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      switch(blue_evt->ecode){
      case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
          evt_gatt_read_permit_req *pr = (void*)blue_evt->data; 
          Read_Request_CB(pr->attr_handle);                    
        }
        break;
      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        if(TargetBoardFeatures.bnrg_expansion_board==IDB05A1) {
              evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
              Attribute_Modified_CB(evt->attr_handle, evt->att_data,evt->data_length);
            } else {
              evt_gatt_attr_modified_IDB04A1 *evt = (evt_gatt_attr_modified_IDB04A1*)blue_evt->data;
              Attribute_Modified_CB(evt->attr_handle, evt->att_data,evt->data_length);
            }
        break;
      }
    }
    break;
  }
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
