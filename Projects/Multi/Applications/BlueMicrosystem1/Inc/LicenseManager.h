/**
  ******************************************************************************
  * @file    LicenseManager.h 
  * @author  Central LAB
  * @version V3.0.0
  * @date    01-June-2016
  * @brief   License Manager APIs
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
#ifndef _LICENSE_MANAGER_H_
#define _LICENSE_MANAGER_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/

/* Exported macros ------------------------------------------------------- */
#define MCR_OSX_COPY_LICENSE_FROM_MANAGER(a,b) memcpy(osx_m##a##_license,&osxLicencesManager.osxMotion##b.osxLicense,3*4*sizeof(int32_t))
#define MCR_OSX_COPY_LICENSE_TO_MANAGER(a,b) memcpy(osxLicencesManager.osxMotion##b.osxLicense,&osx_m##a##_license,3*4*sizeof(int32_t))

/* Exported types ------------------------------------------------------- */

/**
* @brief  License data structure definition
*/
typedef struct {
  uint32_t osxLicenseInitialized;
  char osxLibVersion[36];
  uint32_t osxLicense[3][4];
} osxLicence_t;

/**
* @brief  License Manager data structure definition
*/
typedef struct {
  uint32_t osxLicenseManagerIntialized;

  /* Sensor Fusion Library */
  osxLicence_t osxMotionFX;

  /* Magneto Calibration */
  uint32_t MagnetoCalibration[8];

  /* Carry Position Library */
  osxLicence_t osxMotionCP;

  /* Activity Recognition Library */
  osxLicence_t osxMotionAR;

  /* Gesture Recognition Library */
  osxLicence_t osxMotionGR;

  /* Pedometer Library */
  osxLicence_t osxMotionPM;
} osxLicencesRepository_t;

/**
 * @brief License type
 */
typedef enum
{
  OSX_MOTION_LIC_NULL,
  OSX_MOTION_LIC_FX,
  OSX_MOTION_LIC_AR,
  OSX_MOTION_LIC_CP,
  OSX_MOTION_LIC_GR,
  OSX_MOTION_LIC_PM,
  OSX_MOTION_LIC_NUMBER
} OsxMotionLicenseType_t;

/* Exported defines ------------------------------------------------------- */
#define SIZEOF_OSXLICENSES_MANAGER (sizeof(osxLicencesRepository_t) + 4)

#define OSX_BMS_RESET_LIC (1<<1)
#define OSX_BMS_SAVE_LIC (1)

/* Exported variables ------------------------------------------------------- */
extern osxLicencesRepository_t osxLicencesManager;
extern uint32_t NecessityToSaveLicense;

/* Exported functions ------------------------------------------------------- */
extern void InitLicenseManager(void);
extern unsigned char SaveLicensesStatus(void);
extern unsigned char ResetLicensesStatus(void);
extern void LicensesStatus(void);
extern void ActivateAllLicenses(void);
extern uint32_t LicenseParsingConsoleCommand(uint8_t * att_data, uint8_t data_length);

#ifdef __cplusplus
}
#endif

#endif /* _LICENSE_MANAGER_H_ */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/

