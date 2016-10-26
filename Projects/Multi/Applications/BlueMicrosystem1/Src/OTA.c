/**
  ******************************************************************************
  * @file    OTA.c
  * @author  Central LAB
  * @version V3.0.0
  * @date    01-June-2016
  * @brief   Over-the-Air Update routines
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
#include "OTA.h"

/* Imported variables ---------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/

/* Local defines -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint32_t SizeOfUpdateBlueFW=0;
static uint32_t AspecteduwCRCValue=0;

/* Local function prototypes --------------------------------------------------*/

#if 0
/**
 * @brief Function for Updating the BlueNRG Firmware
 * @param uint32_t fw_size Size of the firmware image.The firmware image size shall
 *                          be multiple of 4 bytes.
 * @retval None
 */
void UpdateFWBlueNRG(uint32_t fw_size)
{
  int32_t ret;
  uint8_t updaterVersion;

  /* Reset BlueNRG hardware */
  BlueNRG_RST();

  OSX_BMS_PRINTF("BlueNRG firmware Updating\r\n");

  if(getBlueNRGUpdaterVersion(&updaterVersion) == BLE_STATUS_SUCCESS) {
    OSX_BMS_PRINTF("BlueNRG Updater version supported\r\n");
    ret = program_device((uint8_t *)OSX_BMS_OTA_ADDRESS_START, fw_size);
    if(ret) {
      OSX_BMS_PRINTF("Error Updating the BlueNRG Error = %d\r\n"
                   "\tWhere:\r\n"
                   "\t\tACI_ERROR=%d\r\n"
                   "\t\tUNSUPPORTED_VERSION=%d\r\n"
                   "\t\tWRONG_IMAGE_SIZE=%d\r\n"
                   "\t\tCRC_ERROR=%d\r\n",
                   ret,
                   BLE_UTIL_ACI_ERROR, BLE_UTIL_UNSUPPORTED_VERSION,
                   BLE_UTIL_WRONG_IMAGE_SIZE, BLE_UTIL_CRC_ERROR);
    } else {
      OSX_BMS_PRINTF("BlueNRG firmware Updated\r\n");
      /* Verify if IFR configuration is already the one
        selected by the user (refer to define values) */
#if 0
      ret = verify_IFR(&IFR_config);
      if (ret) {
        /* IFR configuration is not the selected one: program IFR */
        ret = program_IFR(&IFR_config);
        if (ret) {
          OSX_BMS_PRINTF("Error Configuring IFR\r\n");
        } else {
          OSX_BMS_PRINTF("IFR Configurated\r\n");
        }
      } else {
        OSX_BMS_PRINTF("IFR already OK\r\n");
      }
#endif
      return;
    }
  } else {
    OSX_BMS_PRINTF("BlueNRG Updater version NOT supported\r\n");
  }

  OSX_BMS_PRINTF("BlueNRG Updater version NOT supported\n");
}
#endif
#ifdef USE_STM32F4XX_NUCLEO
/**
 * @brief Function for Updating the BLUEMICROSYSTEM1 Firmware
 * @param uint32_t *SizeOfUpdate Remaining size of the firmware image [bytes].
 * @param uint8_t *att_data attribute data
 * @param int32_t data_length length of the data
 * @param uint8_t WriteMagicNum 1/0 for writing or not the magic number
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
int8_t UpdateFWBlueMS(uint32_t *SizeOfUpdate,uint8_t * att_data, int32_t data_length,uint8_t WriteMagicNum)
{
  int8_t ReturnValue=0;
  /* Save the Packed received */
  if(data_length>(*SizeOfUpdate)){
    /* Too many bytes...Something wrong... necessity to send it again... */
    OSX_BMS_PRINTF("OTA %s something wrong....\r\nPlease Try again\r\n",WriteMagicNum ? "BLUEMICROSYSTEM1" : "BlueNRG");
    ReturnValue = -1;
    /* Reset for Restarting again */
    *SizeOfUpdate=0;
  } else {
    static uint32_t Address = OSX_BMS_OTA_ADDRESS_START;
    int32_t Counter;
    /* Save the received OTA packed ad save it to flash */
    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();
    
    for(Counter=0;Counter<data_length;Counter++) {
      if(HAL_FLASH_Program(TYPEPROGRAM_BYTE, Address,att_data[Counter])==HAL_OK) {
       Address++;
      } else {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error
           FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
        Error_Handler();
      }
    }
    /* Reduce the remaing bytes for OTA completition */
    *SizeOfUpdate -= data_length;

    if(*SizeOfUpdate==0) {
      /* We had received the whole firmware and we have saved it in Flash */

      if(WriteMagicNum) {
        /* Make the CRC integrety check */
        uint32_t uwCRCValue = 0;

        /* CRC handler declaration */
        CRC_HandleTypeDef   CrcHandle;

        /* Init CRC for OTA-integrity check */
        CrcHandle.Instance = CRC;

        if(HAL_CRC_GetState(&CrcHandle) != HAL_CRC_STATE_RESET) {
          HAL_CRC_DeInit(&CrcHandle);
        }

        if (HAL_CRC_Init(&CrcHandle) != HAL_OK) {
          /* Initialization Error */
          Error_Handler();
        } else {
          OSX_BMS_PRINTF("CRC  Initialized\n\r");
        }

        /* Compute the CRC */
        uwCRCValue = HAL_CRC_Calculate(&CrcHandle, (uint32_t *)OSX_BMS_OTA_ADDRESS_START, SizeOfUpdateBlueFW>>2);

        if(uwCRCValue==AspecteduwCRCValue) {
          ReturnValue=1;
          OSX_BMS_PRINTF("OTA BLUEMICROSYSTEM1 CRC-checked\r\n");
          /* We write the Magic number for making the OTA at the next Board reset */
          Address = OSX_BMS_OTA_MAGIC_NUM_POS;

          if(HAL_FLASH_Program(TYPEPROGRAM_WORD, Address,OSX_BMS_OTA_MAGIC_NUM)!=HAL_OK) {
          /* Error occurred while writing data in Flash memory.
             User can add here some code to deal with this error
             FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
          Error_Handler();
          } else {
            OSX_BMS_PRINTF("OTA BLUEMICROSYSTEM1 will be installed at next board reset\r\n");
          }
        } else {
          ReturnValue=-1;
          OSX_BMS_PRINTF("Wrong CRC! Computed=%lx  aspected=%lx ... Try again\r\n",uwCRCValue,AspecteduwCRCValue);
        }
      }
    }

    /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();
  }
  return ReturnValue;
}

/**
 * @brief Start Function for Updating the BLUEMICROSYSTEM1 Firmware
 * @param uint32_t SizeOfUpdate  size of the firmware image [bytes]
 * @param uint32_t uwCRCValue aspected CRV value
 * @retval None
 */
void StartUpdateFWBlueMS(uint32_t SizeOfUpdate, uint32_t uwCRCValue)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  OSX_BMS_PRINTF("Start FLASH Erase\r\n");

  SizeOfUpdateBlueFW = SizeOfUpdate;
  AspecteduwCRCValue = uwCRCValue;

  EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FLASH_SECTOR_6;
  EraseInitStruct.NbSectors = 2;
    
  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK){
    /* Error occurred while sector erase. 
      User can add here some code to deal with this error. 
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
    Error_Handler();
  } else {    
    OSX_BMS_PRINTF("End FLASH Erase\r\n");
  }

  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
}
#elif USE_STM32L4XX_NUCLEO

/**
 * @brief Function for Updating the BLUEMICROSYSTEM1 Firmware
 * @param uint32_t *SizeOfUpdate Remaining size of the firmware image [bytes]
 * @param uint8_t *att_data attribute data
 * @param int32_t data_length length of the data
 * @param uint8_t WriteMagicNum 1/0 for writing or not the magic number
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
int8_t UpdateFWBlueMS(uint32_t *SizeOfUpdate,uint8_t * att_data, int32_t data_length,uint8_t WriteMagicNum)
{
  int8_t ReturnValue=0;
  /* Save the Packed received */
  if(data_length>(*SizeOfUpdate)){
    /* Too many bytes...Something wrong... necessity to send it again... */
    OSX_BMS_PRINTF("OTA BLUEMICROSYSTEM1 something wrong....\r\nPlease Try again\r\n");
    ReturnValue = -1;
    /* Reset for Restarting again */
    *SizeOfUpdate=0;
  } else {
    static uint32_t Address = OSX_BMS_OTA_ADDRESS_START;
    uint64_t ValueToWrite;
    int32_t Counter;
    /* Save the received OTA packed ad save it to flash */
    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();
    
    for(Counter=0;Counter<data_length;Counter+=8) {
      uint8_t *PointerByte = (uint8_t*) &ValueToWrite;
      PointerByte[0] =  att_data[Counter  ];
      
      if((data_length-Counter)>0){
        PointerByte[1] = att_data[Counter+1];
      }
      if((data_length-Counter)>1){
        PointerByte[2] = att_data[Counter+2];
      }
      if((data_length-Counter)>2){
        PointerByte[3] = att_data[Counter+3];
      }
      if((data_length-Counter)>3){
        PointerByte[4] = att_data[Counter+4];
      }
      if((data_length-Counter)>4){
        PointerByte[5] = att_data[Counter+5];
      }
      if((data_length-Counter)>5){
        PointerByte[6] = att_data[Counter+6];
      }
      if((data_length-Counter)>6){
        PointerByte[7] = att_data[Counter+7];
      }
      if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address,ValueToWrite)==HAL_OK) {
       Address+=8;
      } else {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error
           FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
        Error_Handler();
      }
    }
    /* Reduce the remaing bytes for OTA completition */
    *SizeOfUpdate -= data_length;

    if(*SizeOfUpdate==0) {
      /* We had received the whole firmware and we have saved it in Flash */
      OSX_BMS_PRINTF("OTA BLUEMICROSYSTEM1 Update saved\r\n");

      if(WriteMagicNum) {
        /* Make the CRC integrety check */
        uint32_t uwCRCValue = 0;
        /* CRC handler declaration */
        CRC_HandleTypeDef   CrcHandle;

        /* Init CRC for OTA-integrity check */
        CrcHandle.Instance = CRC;
        /* The default polynomial is used */
        CrcHandle.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_ENABLE;

        /* The default init value is used */
        CrcHandle.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE;

        /* The input data are not inverted */
        CrcHandle.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_NONE;

        /* The output data are not inverted */
        CrcHandle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;

        /* The input data are 32-bit long words */
        CrcHandle.InputDataFormat              = CRC_INPUTDATA_FORMAT_WORDS;

        if(HAL_CRC_GetState(&CrcHandle) != HAL_CRC_STATE_RESET) {
          HAL_CRC_DeInit(&CrcHandle);
        }

        if (HAL_CRC_Init(&CrcHandle) != HAL_OK) {
          /* Initialization Error */
          Error_Handler();
        } else {
          OSX_BMS_PRINTF("CRC  Initialized\n\r");
        }

        /* Compute the CRC */
        uwCRCValue = HAL_CRC_Calculate(&CrcHandle, (uint32_t *)OSX_BMS_OTA_ADDRESS_START, SizeOfUpdateBlueFW>>2);

        if(uwCRCValue==AspecteduwCRCValue) {
          ReturnValue=1;
          OSX_BMS_PRINTF("OTA BLUEMICROSYSTEM1 CRC-checked\r\n");
          /* We write the Magic number for making the OTA at the next Board reset */
          Address = OSX_BMS_OTA_MAGIC_NUM_POS;
          ValueToWrite=OSX_BMS_OTA_MAGIC_NUM;

          if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address,ValueToWrite)!=HAL_OK) {
            /* Error occurred while writing data in Flash memory.
               User can add here some code to deal with this error
               FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
            Error_Handler();
          } else {
            OSX_BMS_PRINTF("OTA BLUEMICROSYSTEM1 will be installed at next board reset\r\n");
          }
        } else {
          ReturnValue=-1;
          OSX_BMS_PRINTF("Wrong CRC! Computed=%lx  aspected=%lx ... Try again\r\n",uwCRCValue,AspecteduwCRCValue);
        }
      }
    }

    /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();
  }
  return ReturnValue;
}

/**
 * @brief Start Function for Updating the BLUEMICROSYSTEM1 Firmware
 * @param uint32_t SizeOfUpdate  size of the firmware image [bytes]
 * @param uint32_t uwCRCValue aspected CRV value
 * @retval None
 */
void StartUpdateFWBlueMS(uint32_t SizeOfUpdate, uint32_t uwCRCValue)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  OSX_BMS_PRINTF("Start FLASH Erase\r\n");

  SizeOfUpdateBlueFW = SizeOfUpdate;
  AspecteduwCRCValue = uwCRCValue;

  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = GetBank(OSX_BMS_OTA_MAGIC_NUM_POS);
  EraseInitStruct.Page        = GetPage(OSX_BMS_OTA_MAGIC_NUM_POS);
  EraseInitStruct.NbPages     = OSX_BMS_MAX_PROG_SIZE/FLASH_PAGE_SIZE;
    
  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK){
    /* Error occurred while sector erase. 
      User can add here some code to deal with this error. 
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
    Error_Handler();
  } else {    
    OSX_BMS_PRINTF("End FLASH Erase\r\n");
  }

  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
}
#endif /* USE_STM32F4XX_NUCLEO */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
