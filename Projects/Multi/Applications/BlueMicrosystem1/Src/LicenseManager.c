/**
  ******************************************************************************
  * @file    LicenseManager.c
  * @author  Central LAB
  * @version V3.0.0
  * @date    01-June-2016
  * @brief   License Manager APIs implementation
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
#include "LicenseManager.h"
#include "main.h"
#include "sensor_service.h"

/* Imported variables ---------------------------------------------------------*/
extern uint32_t osx_mfx_license[3][4];
#ifdef OSX_BMS_MOTIONCP
extern uint32_t osx_mcp_license[3][4];
#endif /* OSX_BMS_MOTIONCP */

#ifdef OSX_BMS_MOTIONAR
extern uint32_t osx_mar_license[3][4];
#endif /* OSX_BMS_MOTIONAR */

#ifdef OSX_BMS_MOTIONGR
extern uint32_t osx_mgr_license[3][4];
#endif /* OSX_BMS_MOTIONGR */

#ifdef OSX_BMS_MOTIONPM
extern uint32_t osx_mpm_license[3][4];
#endif /* OSX_BMS_MOTIONPM */

/* Exported variables ---------------------------------------------------------*/
osxLicencesRepository_t osxLicencesManager;
uint32_t NecessityToSaveLicense=0;

/* Local defines -------------------------------------------------------------*/
#define BLUEMSYS_CHECK_LIC_MANAGER_INIT ((uint32_t)0x12345678)

#ifdef USE_STM32F4XX_NUCLEO
/* Inside of Sector 7 */
#define BLUEMSYS_FLASH_ADD ((uint32_t)(0x0807FFFF-0x3FFF))
#define BLUEMSYS_FLASH_SECTOR FLASH_SECTOR_7
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
/* Base @ of Page 511, 2 Kbytes */
#define BLUEMSYS_FLASH_ADD ((uint32_t)0x080ff800)
#endif /* USE_STM32L4XX_NUCLEO */

/* Local Macros ---------------------------------------------------------------*/
#define OSX_OSX_PRINT_LIC_STATUS(Name) {\
  BytesToWrite =sprintf((char *)BufferToWrite,"osxMotion" #Name" %s\r\n",\
                      (osxLicencesManager.osxMotion##Name.osxLicenseInitialized ? "OK" : "--"));\
  Term_Update(BufferToWrite,BytesToWrite);\
}

#define OSX_OSX_ACTIVATE_LIC(Name) {\
  if(osxLicencesManager.osxMotion##Name.osxLicenseInitialized==0) {\
    Motion##Name##_License_init();\
  } else {\
    BytesToWrite =sprintf((char *)BufferToWrite,"osxMotion" #Name" Already Initialized\r\n");\
    Term_Update(BufferToWrite,BytesToWrite);\
  }\
}

#define OSX_OSX_START_ACTIVATE_LIC(Name) {\
if(osxLicencesManager.osxMotion##Name.osxLicenseInitialized==0) {\
    osxMotionLicStart=1;\
    osxMotionLicCount=0;\
    OsxMotionLicense=OSX_MOTION_LIC_##Name;\
    {\
      int32_t counter;\
      for(counter=2;counter<data_length;counter++) {\
         TmpLicense[osxMotionLicCount]=att_data[counter];\
         osxMotionLicCount++;\
        }\
    }\
  } else {\
    BytesToWrite =sprintf((char *)BufferToWrite,"osxMotion" #Name " Already Initialized\r\n");\
    Term_Update(BufferToWrite,BytesToWrite);\
  }\
}

#define OSX_OSX_BODY_ACTIVATE_LIC(lowName,UPName) {\
  {\
    int32_t counter;\
    for(counter=0;counter<data_length;counter++) {\
       TmpLicense[osxMotionLicCount]=att_data[counter];\
       osxMotionLicCount++;\
      }\
  }\
  /* if we had received enough License's bytes ...*/\
  if(osxMotionLicCount== (3*4*4)){\
    osxMotionLicCount=0;\
    osxMotionLicStart=0;\
    OsxMotionLicense=OSX_MOTION_LIC_NULL;\
    /* Swap endianess */\
    {\
      int32_t Counter;\
      uint8_t * DestLicPointer = ((uint8_t *) osxLicencesManager.osxMotion##UPName.osxLicense);\
      for(Counter=0;Counter<3*4*4;Counter+=4) {\
        DestLicPointer[Counter+3] = TmpLicense[Counter+0];\
        DestLicPointer[Counter+0] = TmpLicense[Counter+3];\
        DestLicPointer[Counter+2] = TmpLicense[Counter+1];\
        DestLicPointer[Counter+1] = TmpLicense[Counter+2];\
      }\
    }\
    MCR_OSX_COPY_LICENSE_FROM_MANAGER(lowName,UPName);\
    Motion##UPName##_License_init();\
    BytesToWrite =sprintf((char *)BufferToWrite,"License Write Successful\r\n");\
    Term_Update(BufferToWrite,BytesToWrite);\
  }\
}

/* Local function prototypes --------------------------------------------------*/
static unsigned char ReCallLicensesStatus(void);

/**
  * @brief  Initialize the osxMotion Licenses manager
  * @param  None
  * @retval None
  */
void InitLicenseManager(void)
{ 
  NecessityToSaveLicense=0;
  
  //ResetLicensesStatus();
  
  /* Read the osxMotion Licenses status from FLASH */
  ReCallLicensesStatus();

  if(osxLicencesManager.osxLicenseManagerIntialized==0) {

    OSX_BMS_PRINTF("FLASH doesn't contain a Initialized osx Licenses Manager\r\n");
  
    sprintf((char *)osxLicencesManager.osxMotionFX.osxLibVersion,"osxMotionFX x9/x6 v1.0.7");

#ifdef OSX_BMS_MOTIONAR
    sprintf((char *)osxLicencesManager.osxMotionAR.osxLibVersion,"osxMotionAR v1.1.0");
#endif /* OSX_BMS_MOTIONAR */

#ifdef OSX_BMS_MOTIONCP
    sprintf((char *)osxLicencesManager.osxMotionCP.osxLibVersion,"osxMotionCP v1.0.0");
#endif /* OSX_BMS_MOTIONCP */

#ifdef OSX_BMS_MOTIONGR
    sprintf((char *)osxLicencesManager.osxMotionGR.osxLibVersion,"osxMotionGR v1.0.0");
#endif /* OSX_BMS_MOTIONGR */

#ifdef OSX_BMS_MOTIONPM
    sprintf((char *)osxLicencesManager.osxMotionPM.osxLibVersion,"osxMotionPM v1.0.0");
#endif /* OSX_BMS_MOTIONPM */

#ifdef OSX_BMS_LICENSE_H_FILE
    /* Initialize MotionFX License */
    MotionFX_License_init();

#ifdef OSX_BMS_MOTIONAR
    /* Initialize MotionAR License */
    MotionAR_License_init();
#endif /* OSX_BMS_MOTIONAR */

#ifdef OSX_BMS_MOTIONCP
    /* Initialize MotionCP License */
    MotionCP_License_init();
#endif /* OSX_BMS_MOTIONCP */

#ifdef OSX_BMS_MOTIONGR
    /* Initialize MotionGR License */
    MotionGR_License_init();
#endif /* OSX_BMS_MOTIONGR */

#ifdef OSX_BMS_MOTIONPM
    /* Initialize MotionPM License */
    MotionPM_License_init();
#endif /* OSX_BMS_MOTIONPM */    
#endif /* OSX_BMS_LICENSE_H_FILE */
  } else {

     OSX_BMS_PRINTF("FLASH contains a Initialized osx Licenses Manager\r\n");

    /* Read the osxMotion Licenses from License Manager */
    MCR_OSX_COPY_LICENSE_FROM_MANAGER(fx,FX);
#ifdef OSX_BMS_MOTIONAR
    MCR_OSX_COPY_LICENSE_FROM_MANAGER(ar,AR);
#endif /* OSX_BMS_MOTIONAR */
#ifdef OSX_BMS_MOTIONCP
    MCR_OSX_COPY_LICENSE_FROM_MANAGER(cp,CP);
#endif /* OSX_BMS_MOTIONCP */
#ifdef OSX_BMS_MOTIONGR
    MCR_OSX_COPY_LICENSE_FROM_MANAGER(gr,GR);
#endif /* OSX_BMS_MOTIONGR */
#ifdef OSX_BMS_MOTIONPM
    MCR_OSX_COPY_LICENSE_FROM_MANAGER(pm,PM);
#endif /* OSX_BMS_MOTIONPM */
    /* osxMotionFX License */
    if(osxLicencesManager.osxMotionFX.osxLicenseInitialized) {
      /* Initialize MotionFX License */
      MotionFX_License_init();
    }
#ifdef OSX_BMS_MOTIONAR
    /* osxMotionAR License */
    if(osxLicencesManager.osxMotionAR.osxLicenseInitialized) {
      /* Initialize MotionAR License */
      MotionAR_License_init();
    }
#endif /* OSX_BMS_MOTIONAR */

#ifdef OSX_BMS_MOTIONCP
    /* osxMotionCP License */
    if(osxLicencesManager.osxMotionCP.osxLicenseInitialized) {
      /* Initialize MotionCP License */
      MotionCP_License_init();
    }
#endif /* OSX_BMS_MOTIONCP */

#ifdef OSX_BMS_MOTIONGR
    /* osxMotionGR License */
    if(osxLicencesManager.osxMotionGR.osxLicenseInitialized) {
      /* Initialize MotionGR License */
      MotionGR_License_init();
    }
#endif /* OSX_BMS_MOTIONGR */

#ifdef OSX_BMS_MOTIONPM
    /* osMotionPM License */
    if(osxLicencesManager.osxMotionPM.osxLicenseInitialized) {
      /* Initialize MotionPM License */
      MotionPM_License_init();
    }
#endif /* OSX_BMS_MOTIONPM */
  }
}

/**
 * @brief  Save the osxMotion licenses status to Flash
 * @param None
 * @retval unsigned char Success/Not Success
 */
unsigned char SaveLicensesStatus(void)
{
  unsigned char Success=1;

  if(Success) {
#ifdef USE_STM32F4XX_NUCLEO
    /* Store in Flash Memory */
    uint32_t Address = BLUEMSYS_FLASH_ADD;
    uint32_t WriteArray[1+(sizeof(osxLicencesRepository_t)>>2)];
    int32_t WriteIndex;
    WriteArray[0] = BLUEMSYS_CHECK_LIC_MANAGER_INIT;
    memcpy(WriteArray+1,&osxLicencesManager,sizeof(osxLicencesRepository_t));    
   /* Unlock the Flash to enable the flash control register access *************/
   HAL_FLASH_Unlock();

    for(WriteIndex=0;WriteIndex<(1+(sizeof(osxLicencesRepository_t)>>2));WriteIndex++){
      if (HAL_FLASH_Program(TYPEPROGRAM_WORD, Address,WriteArray[WriteIndex]) == HAL_OK){
        Address = Address + 4;
      } else {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error
           FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
        Error_Handler();
      }
    }
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
    /* Store in Flash Memory */
    uint32_t Address = BLUEMSYS_FLASH_ADD;
    uint64_t WriteArray64[1+(sizeof(osxLicencesRepository_t)>>3)];
    uint32_t *WriteArray32 = (uint32_t*) WriteArray64;
    int32_t WriteIndex;
    WriteArray32[0] = BLUEMSYS_CHECK_LIC_MANAGER_INIT;
    memcpy(WriteArray32+1,&osxLicencesManager,sizeof(osxLicencesRepository_t));

   /* Unlock the Flash to enable the flash control register access *************/
   HAL_FLASH_Unlock();

    for(WriteIndex=0;WriteIndex<(1+(sizeof(osxLicencesRepository_t)>>3));WriteIndex++){
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address,WriteArray64[WriteIndex]) == HAL_OK){
        Address = Address + 8;
      } else {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error
           FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
        Error_Handler();
      }
    }
#endif /* USE_STM32L4XX_NUCLEO */

    OSX_BMS_PRINTF("osxMotion Licenses status saved in FLASH\r\n");

    /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();
  }
  return Success;
}

/**
 * @brief  Reset the osxMotion licenses status in Flash
 * @param None
 * @retval unsigned char Success/Not Success
 */
unsigned char ResetLicensesStatus(void)
{
  /* Reset Licenses Value in FLASH */
  unsigned char Success=1;

  /* Erase First Flash sector */
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;

#ifdef USE_STM32F4XX_NUCLEO
  EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = BLUEMSYS_FLASH_SECTOR;
  EraseInitStruct.NbSectors = 1;
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = GetBank(BLUEMSYS_FLASH_ADD);
  EraseInitStruct.Page        = GetPage(BLUEMSYS_FLASH_ADD);
  EraseInitStruct.NbPages     = 1;
#endif /* USE_STM32L4XX_NUCLEO */

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK){
    /* Error occurred while sector erase. 
      User can add here some code to deal with this error. 
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
    Error_Handler();
    Success=0;
  } else {    
    OSX_BMS_PRINTF("osxMotion Licenses status erased in FLASH\r\n");
  }

  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  return Success;
}

/**
 * @brief  Check if there is a valid osxMotion licenses status in Flash and read it
 * @param None
 * @retval unsigned char Success/Not Success
 */
static unsigned char ReCallLicensesStatus(void)
{
  /* ReLoad the Licenses Value from FLASH */
  unsigned char Success=1;
  uint32_t Address = BLUEMSYS_FLASH_ADD;
  __IO uint32_t data32 = *(__IO uint32_t*) Address;

  if(data32== BLUEMSYS_CHECK_LIC_MANAGER_INIT){
    int32_t ReadIndex;
    uint32_t *ReadArray = (uint32_t *)&osxLicencesManager;

    for(ReadIndex=0;ReadIndex<(sizeof(osxLicencesRepository_t)>>2);ReadIndex++){
      Address +=4;
      data32 = *(__IO uint32_t*) Address;
      ReadArray[ReadIndex]=data32;
    }
    
    /* Set the Calibration Structure */
    OSX_BMS_PRINTF("osxMotion Licenses status read from Flash\r\n");
  } else {
    OSX_BMS_PRINTF("osxMotion Licenses status not present in FLASH\r\n");
  }
  return Success;
}


/**
 * @brief  This function prints the osxMotion licenses status
 * @param None
 * @retval None
 */
void LicensesStatus(void)
{
  OSX_OSX_PRINT_LIC_STATUS(FX);

#ifdef OSX_BMS_MOTIONAR
  OSX_OSX_PRINT_LIC_STATUS(AR);
#endif /* OSX_BMS_MOTIONAR */

#ifdef OSX_BMS_MOTIONCP
  OSX_OSX_PRINT_LIC_STATUS(CP);
#endif /* OSX_BMS_MOTIONCP */

#ifdef OSX_BMS_MOTIONGR
  OSX_OSX_PRINT_LIC_STATUS(GR);
#endif /* OSX_BMS_MOTIONGR */

#ifdef OSX_BMS_MOTIONPM
    OSX_OSX_PRINT_LIC_STATUS(PM);
#endif /* OSX_BMS_MOTIONPM */
}


/**
 * @brief  This function activates all the osxMotion License availables
 * @param None
 * @retval None
 */
void ActivateAllLicenses(void)
{
  OSX_OSX_ACTIVATE_LIC(FX);

#ifdef OSX_BMS_MOTIONAR
  OSX_OSX_ACTIVATE_LIC(AR);
#endif /* OSX_BMS_MOTIONAR */

#ifdef OSX_BMS_MOTIONCP
  OSX_OSX_ACTIVATE_LIC(CP);
#endif /* OSX_BMS_MOTIONCP */

#ifdef OSX_BMS_MOTIONGR
  OSX_OSX_ACTIVATE_LIC(GR);
#endif /* OSX_BMS_MOTIONGR */

#ifdef OSX_BMS_MOTIONPM
  OSX_OSX_ACTIVATE_LIC(PM);
#endif /* OSX_BMS_MOTIONPM */
}

/**
 * @brief  This function makes the parsing of the Debug Console Commands for osxMotion License management
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval uint32_t osxMotionLicStart 1/0 if we are o not activating a licenses
 */
uint32_t LicenseParsingConsoleCommand(uint8_t * att_data, uint8_t data_length)
{
  static uint32_t osxMotionLicStart=0;
  static uint32_t osxMotionLicCount=0;
  static  OsxMotionLicenseType_t OsxMotionLicense=OSX_MOTION_LIC_NULL;
  static uint8_t TmpLicense[4*3*4];

  if(osxMotionLicStart==0){
    /* No osxMotion License activation phase */
    if((att_data[0]=='F') & (att_data[1]=='X')) {
      OSX_OSX_START_ACTIVATE_LIC(FX);
    }
#ifdef OSX_BMS_MOTIONAR
    else if((att_data[0]=='A') & (att_data[1]=='R')) {
      OSX_OSX_START_ACTIVATE_LIC(AR);
    }
#endif /* OSX_BMS_MOTIONAR */
#ifdef OSX_BMS_MOTIONCP
    else if((att_data[0]=='C') & (att_data[1]=='P')) {
      OSX_OSX_START_ACTIVATE_LIC(CP);
    }
#endif /* OSX_BMS_MOTIONCP */
#ifdef OSX_BMS_MOTIONGR
    else if((att_data[0]=='G') & (att_data[1]=='R')) {
      OSX_OSX_START_ACTIVATE_LIC(GR);
    }
#endif /* OSX_BMS_MOTIONGR */
#ifdef OSX_BMS_MOTIONPM
    else if((att_data[0]=='P') & (att_data[1]=='M')) {
      OSX_OSX_START_ACTIVATE_LIC(PM);
    }
#endif /* OSX_BMS_MOTIONPM */
  } else {
    /* osxMotion License activation phase */
    switch(OsxMotionLicense) {
      case OSX_MOTION_LIC_FX:
        OSX_OSX_BODY_ACTIVATE_LIC(fx,FX);
      break;
#ifdef OSX_BMS_MOTIONAR
      case OSX_MOTION_LIC_AR:
        OSX_OSX_BODY_ACTIVATE_LIC(ar,AR);
      break;
#endif /* OSX_BMS_MOTIONAR */
#ifdef OSX_BMS_MOTIONCP
      case OSX_MOTION_LIC_CP:
        OSX_OSX_BODY_ACTIVATE_LIC(cp,CP);
      break;
#endif /* OSX_BMS_MOTIONCP */
#ifdef OSX_BMS_MOTIONGR
      case OSX_MOTION_LIC_GR:
        OSX_OSX_BODY_ACTIVATE_LIC(gr,GR);
      break;
#endif /* OSX_BMS_MOTIONGR */
#ifdef OSX_BMS_MOTIONPM
      case OSX_MOTION_LIC_PM:
        OSX_OSX_BODY_ACTIVATE_LIC(pm,PM);
      break;
#endif /* OSX_BMS_MOTIONPM */
      default:
        BytesToWrite =sprintf((char *)BufferToWrite,"Error Initializing osxMotion License\r\n");
        Term_Update(BufferToWrite,BytesToWrite);  
      break;
    }
  }
  return osxMotionLicStart;
}

#undef OSX_OSX_ACTIVATE_LIC
#undef OSX_OSX_BODY_ACTIVATE_LIC
#undef OSX_OSX_START_ACTIVATE_LIC
#undef OSX_OSX_PRINT_LIC_STATUS

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
