/**
  ******************************************************************************
  * @file ADPCM/inc/adpcm.h 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  Header file for adpcm.c
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADPCM_H
#define __ADPCM_H

#define ADPCM_AUDIO_DATA_COUNT	16144
#define ADPCM_NB_AUDIO_FILES    1
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Exported types ------------------------------------------------------------*/
typedef union
{
	uint8_t  uBytes[2];
	uint16_t uShort;
} tTwoByte;

typedef struct
{
	uint32_t AudioFiles[ADPCM_NB_AUDIO_FILES];
	uint32_t AudioSize[ADPCM_NB_AUDIO_FILES];
} AudioElement;

/* Exported constants --------------------------------------------------------*/
extern const unsigned char rawData[ADPCM_AUDIO_DATA_COUNT];
/* Exported macro ------------------------------------------------------------*/
#define ADPCM_NELEMS(x)  (sizeof(x) / sizeof((x)[0]))
/* Exported functions ------------------------------------------------------- */
uint8_t ADPCM_Encode(int32_t sample);
int16_t ADPCM_Decode(uint8_t code);

#endif /* __ADPCM_H*/
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
