/**
  ******************************************************************************
  * @file    Audio_playback_and_record/src/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    28-October-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tm_stm32f4_i2c.h"
#include "skywriter_play.h"
#include "stm32f4_discovery_audio_codec.h"
#include <math.h>     
#include "arm_math.h" 
#define SAMPLING_FREQ           (22000)
extern int16_t AUDIO_SAMPLE[];
/** @addtogroup STM32F4-Discovery_Audio_Player_Recorder
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if defined MEDIA_USB_KEY
 USB_OTG_CORE_HANDLE          USB_OTG_Core;
 USBH_HOST                    USB_Host;
#endif

RCC_ClocksTypeDef RCC_Clocks;
__IO uint8_t RepeatState = 0;
__IO uint16_t CCR_Val = 16826;
float modulation_frequency = 1000.0;
float modulation_intensity = 1.0;
float signal_level = 0.8;
float theta = 0.0f;
float theta_increment;
float modulated_result;
extern __IO uint8_t LED_Toggle;

/* Private function prototypes -----------------------------------------------*/
static void TIM_LED_Config(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
*/

void cs43l22_init(void){
    GPIO_InitTypeDef gpio;

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = SPI3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    //CS43L22 /RESET(PD4)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    gpio.GPIO_Pin=GPIO_Pin_4;
    gpio.GPIO_Mode=GPIO_Mode_OUT;
    gpio.GPIO_OType=GPIO_OType_PP;
    gpio.GPIO_PuPd=GPIO_PuPd_DOWN;
    gpio.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);

    //CS43L22 I2C SDA(PB9) in SCL(PB6)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    gpio.GPIO_Pin=GPIO_Pin_6 | GPIO_Pin_9;
    gpio.GPIO_Mode=GPIO_Mode_AF;
    gpio.GPIO_OType=GPIO_OType_OD;
    gpio.GPIO_PuPd=GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

    //CS43L22 I2S3 WS(PA4);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin=GPIO_Pin_4;
    gpio.GPIO_Mode=GPIO_Mode_AF;
    gpio.GPIO_OType=GPIO_OType_PP;
    gpio.GPIO_PuPd=GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI3);

    //CS43L22 I2S3 MCK(PC7), SCK(PC10), SD(PC12)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    gpio.GPIO_Pin=GPIO_Pin_7 | GPIO_Pin_10 | GPIO_Pin_12;
    gpio.GPIO_Mode=GPIO_Mode_AF;
    gpio.GPIO_OType=GPIO_OType_PP;
    gpio.GPIO_PuPd=GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);

    //I2S config
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
    SPI_DeInit(SPI3);
    I2S_InitTypeDef i2s;
    i2s.I2S_AudioFreq=I2S_AudioFreq_8k;
    i2s.I2S_MCLKOutput=I2S_MCLKOutput_Enable;
    i2s.I2S_Mode=I2S_Mode_MasterTx;
    i2s.I2S_DataFormat=I2S_DataFormat_16b;
    i2s.I2S_Standard=I2S_Standard_Phillips;
    i2s.I2S_CPOL=I2S_CPOL_Low;
    I2S_Init(SPI3, &i2s);
    I2S_Cmd(SPI3, ENABLE);

    //I2C1 config
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    I2C_InitTypeDef i2c;
    i2c.I2C_ClockSpeed=100000;
    i2c.I2C_Mode=I2C_Mode_I2C;
    i2c.I2C_Ack=I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
    i2c.I2C_DutyCycle=I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1=99;
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);

    //CS43L22 config
    //Recommended Power-up sequence (page 31)

    //bring reset high
    GPIO_SetBits(GPIOD, GPIO_Pin_4);

    //wait
    volatile uint32_t j;
    for(j=0; j<20000000; j++);

    //Required initialization settings (page 32)
    uint8_t reg;
    Codec_WriteRegister(0x0, 0x99);
    Codec_WriteRegister(0x47, 0x80);
    reg = Codec_ReadRegister(0x32);
    Codec_WriteRegister(0x32, reg | 0x80);
    reg = Codec_ReadRegister(0x32);
    Codec_WriteRegister(0x32, reg & 0x7F);
    Codec_WriteRegister(0x0, 0x00);

    Codec_WriteRegister(0x02, 0x01);
    Codec_WriteRegister(0x04, 0xAF);
    Codec_WriteRegister(0x05, 0x80);
    Codec_WriteRegister(0x06, 0x04);//0x07);
    Codec_WriteRegister(0x02, 0x9E);

    Codec_WriteRegister(0x0A, 0x00);
    Codec_WriteRegister(0x0E, 0x04);
    Codec_WriteRegister(0x27, 0x00);
    Codec_WriteRegister(0x1F, 0x0F);

    Codec_WriteRegister(0x1A, 0x7F);
    Codec_WriteRegister(0x1B, 0x7F);
}

#define FLIP_FLOP_TAP   (15)

typedef enum
{
  SW_EVT_NOTHING = 0x00,
  SW_EVT_XYZ = 0x01,
  SW_EVT_TOUCHED = 0x02
} skywriterEvent_t;

typedef enum
{
  SW_STATE_TOUCHED = 0x01,
  SW_STATE_RELEASED = 0x02
} skywriterState_t;

skywriterEvent_t skywriterEventPoll()
{
  static skywriterState_t currentState = SW_STATE_RELEASED;
  static uint16_t nonTouchCounter = 0;
  skywriterEvent_t returnEvent = SW_EVT_NOTHING;
  packetType_t currentPacketType = NB_skywriter_poll();
  
  if (currentPacketType & PACKET_XYZ) returnEvent |= SW_EVT_XYZ;
  
  switch (currentState)
  {
    case SW_STATE_RELEASED:
      if (currentPacketType & PACKET_TOUCH)
      {
        nonTouchCounter = 0;
        returnEvent |= SW_EVT_TOUCHED;
        currentState = SW_STATE_TOUCHED;
      }
      break;
    case SW_STATE_TOUCHED:
      if (currentPacketType & PACKET_TOUCH)
      {
        nonTouchCounter = 0;
      }
      else if (currentPacketType & PACKET_XYZ)
      {
        if (++nonTouchCounter == FLIP_FLOP_TAP)
        {
          currentState = SW_STATE_RELEASED;
        }
      }
      break;
    default:
      break;
  }
}

typedef enum
{
  PLAYER_STATE_PLAY = 0x00,
  PLAYER_STATE_STOP = 0x01
} playerState_t;

#define SONG_LENGTH 70000

int main(void)
{ 
  /* Initialize LEDS */
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);
 
  /* Green Led On: start of application */
  STM_EVAL_LEDOn(LED4);
       
  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
  
  /* Configure TIM4 Peripheral to manage LEDs lighting */
  TIM_LED_Config();
  //TM_I2C_Init(I2C1, TM_I2C_PinsPack_1, 100000);
  //connected = TM_I2C_IsDeviceConnected(I2C1, 0x42);
  
  skywriter_init(GPIO_PIN_6, GPIO_PIN_5);
  /* Initialize the repeat status */
  RepeatState = 0;
  LED_Toggle = 7;
  
#if defined MEDIA_IntFLASH
  cs43l22_init();
  int iplay = 0;
  float non_modulated_factor;// = 1.0 - modulation_intensity;
  Delay(10);
  unsigned int x,y,z;
  unsigned long counter = 0;
  playerState_t playerState = PLAYER_STATE_STOP;
  while (1)
  {
    skywriterEvent_t currentSWEvent = skywriterEventPoll();
    if (currentSWEvent & SW_EVT_XYZ)
    {
      getXYZ(&x, &y, &z);
      modulation_frequency = (z / 65);
      modulation_intensity = ((float)x/65535.0);
      signal_level = ((float)y/65535.0);
      non_modulated_factor = 1.0 - modulation_intensity;
      theta_increment = 2*3.14*modulation_frequency/SAMPLING_FREQ;   
    }
    switch (playerState)
    {
      case PLAYER_STATE_STOP:
        if (currentSWEvent & SW_EVT_TOUCHED)
        {
          // Play from the beginning of the song
          iplay = 0;
          playerState = PLAYER_STATE_PLAY;
        }
        break;
      case PLAYER_STATE_PLAY:
        if (currentSWEvent & SW_EVT_TOUCHED)
        {
          // Play from the beginning if touched while playing
          iplay = 0;
        }
        else
        {
          // else send the file to continue playing
          if(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE))
          {
            theta += theta_increment;
            {
              theta -= 2*3.14;
            }
            modulated_result = (((AUDIO_SAMPLE[iplay])*non_modulated_factor+ (AUDIO_SAMPLE[iplay])*modulation_intensity*arm_sin_f32(theta))*signal_level);
            SPI_I2S_SendData(SPI3, modulated_result);
            if (++iplay == SONG_LENGTH)
            {
              playerState = PLAYER_STATE_STOP;
            }
          }
        }
    }
  }
  
#elif defined MEDIA_USB_KEY
  
  /* Initialize User Button */
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
   
  /* Init Host Library */
  USBH_Init(&USB_OTG_Core, USB_OTG_FS_CORE_ID, &USB_Host, &USBH_MSC_cb, &USR_Callbacks);
  
  while (1)
  {
    /* Host Task handler */
    USBH_Process(&USB_OTG_Core, &USB_Host);
  }
  
#endif
  
}

/**
  * @brief  Configures the TIM Peripheral for Led toggling.
  * @param  None
  * @retval None
  */
static void TIM_LED_Config(void)
{
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  uint16_t prescalervalue = 0;
  
  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Initialize Leds mounted on STM324F4-EVAL board */
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED6);
  
  /* Compute the prescaler value */
  prescalervalue = (uint16_t) ((SystemCoreClock ) / 550000) - 1;
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = prescalervalue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  /* Enable TIM4 Preload register on ARR */
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  
  /* TIM PWM1 Mode configuration: Channel */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  
  /* Output Compare PWM1 Mode configuration: Channel2 */
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable);
    
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM4, TIM_IT_CC1 , ENABLE);
  
  /* TIM4 enable counter */
  TIM_Cmd(TIM4, ENABLE);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
