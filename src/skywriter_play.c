#include "skywriter_play.h"
#include "stm32f4xx.h"
#include "tm_stm32f4_i2c.h"
/*
* Hardware-specific local variables
*/
GPIO_InitTypeDef Xfer_output;
GPIO_InitTypeDef Xfer_input;
GPIO_InitTypeDef Reset_output;

uint16_t xfer_pin;
uint16_t reset_pin;
I2C_InitTypeDef hi2c1;


/*
* Driver local variables
*/
unsigned char buffer[32];
unsigned int x,y,z;
unsigned char last_gesture, last_touch;
int rotation;
int lastrotation;

/*
* Driver Macros
*/
#define SW_ADDR 0x42

#define SW_HEADER_SIZE   4

#define SW_DATA_DSP      1 //0b0000000000000001
#define SW_DATA_GESTURE  1 << 1 //0b0000000000000010
#define SW_DATA_TOUCH    1 << 2 //0b0000000000000100
#define SW_DATA_AIRWHEEL 1 << 3 //0b0000000000001000
#define SW_DATA_XYZ      1 << 4 //0b0000000000010000

#define SW_SYSTEM_STATUS 0x15
#define SW_REQUEST_MSG   0x06
#define SW_FW_VERSION    0x83
#define SW_SET_RUNTIME   0xA2
#define SW_SENSOR_DATA   0x91

#define SW_PAYLOAD_HDR_CONFIGMASK  0 // 2 Bytes
#define SW_PAYLOAD_HDR_TS          2 // 1 Byte
#define SW_PAYLOAD_HDR_SYSINFO     3 // 1 Byte
#define SW_PAYLOAD_DSP_STATUS      4
#define SW_PAYLOAD_GESTURE         6  // 4 Bytes
#define SW_PAYLOAD_TOUCH           10 // 4 Bytes
#define SW_PAYLOAD_AIRWHEEL        14 // 2 Bytes
#define SW_PAYLOAD_X               16 // 2 bytes
#define SW_PAYLOAD_Y               18 // 2 bytes
#define SW_PAYLOAD_Z               20 // 2 bytes

#define SW_SYS_POSITION            1
#define SW_SYS_AIRWHEEL            1 << 1

/*
Constants for identifying tap
*/
#define SW_DOUBLETAP_CENTER        0
#define SW_DOUBLETAP_EAST          1
#define SW_DOUBLETAP_NORTH         2
#define SW_DOUBLETAP_WEST          3
#define SW_DOUBLETAP_SOUTH         4
#define SW_TAP_CENTER              5
#define SW_TAP_EAST                6
#define SW_TAP_NORTH               7
#define SW_TAP_WEST                8
#define SW_TAP_SOUTH               9
#define SW_TOUCH_CENTER            10
#define SW_TOUCH_EAST              11
#define SW_TOUCH_NORTH             12
#define SW_TOUCH_WEST              13
#define SW_TOUCH_SOUTH             14

#define SW_GESTURE_GARBAGE         1
#define SW_FLICK_WEST_EAST         2
#define SW_FLICK_EAST_WEST         3
#define SW_FLICK_SOUTH_NORTH       4
#define SW_FLICK_NORTH_SOUTH       5
#define SW_CIRCLE_CLOCKWISE        6
#define SW_CIRCLE_CCLOCKWISE       7



// Function prototypes
static void MX_I2C1_Init(void);
static packetType_t handle_sensor_data(unsigned char* data);

static void skywriterDelay(unsigned long inputCycle)
{
  unsigned long counter = 0;
  while (counter < inputCycle)
  {
    counter++;
  }
}

void skywriter_init(uint16_t xferPin, uint16_t resetPin)
{
  xfer_pin = xferPin;
  reset_pin = resetPin;
  //MX_I2C1_Init();
  TM_I2C_Init(I2C1, TM_I2C_PinsPack_1, 200000);
  /* USER CODE BEGIN 2 */
  Xfer_output.GPIO_Pin = xfer_pin;
  Xfer_output.GPIO_Mode = GPIO_Mode_OUT;
  Xfer_output.GPIO_PuPd = GPIO_PuPd_NOPULL;
  Xfer_output.GPIO_OType = GPIO_OType_PP;
  Xfer_output.GPIO_Speed = GPIO_Speed_2MHz;
  
  TM_GPIO_INT_EnableClock(GPIOD);
  Reset_output.GPIO_Pin = reset_pin;
  Reset_output.GPIO_Mode = GPIO_Mode_OUT;
  Reset_output.GPIO_PuPd = GPIO_PuPd_NOPULL;
  Reset_output.GPIO_OType = GPIO_OType_PP;
  Reset_output.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOD, &Reset_output);
  
  Xfer_input.GPIO_Pin = xfer_pin;
  Xfer_input.GPIO_Mode = GPIO_Mode_IN;
  Xfer_input.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &Xfer_input);
  
  //HAL_GPIO_WritePin(GPIOD, reset_pin, GPIO_PIN_RESET);
  GPIO_WriteBit(GPIOD, reset_pin, Bit_RESET);
  //HAL_Delay(100);
  skywriterDelay(1000000);
  GPIO_WriteBit(GPIOD, reset_pin, Bit_SET);
  //HAL_Delay(100);
  skywriterDelay(1000000);
}

packetType_t skywriter_poll()
{
  packetType_t thisPacketType =  PACKET_NOTHING;
  if (GPIO_ReadInputDataBit(GPIOD, xfer_pin) == Bit_RESET)
  {
    GPIO_Init(GPIOD, &Xfer_output);
    GPIO_WriteBit(GPIOD, xfer_pin, Bit_RESET);
    //HAL_I2C_Master_Receive(&hi2c1,SW_ADDR<<1, buffer , 32 , 50);
    TM_I2C_ReadMultiNoRegister(I2C1, SW_ADDR << 1, buffer, 32);
    unsigned char size, flag, seq, ident;
    size = buffer[0];
    flag = buffer[1];
    seq = buffer[2];
    ident = buffer[3];
    switch (ident)
    {
    case 0x91:
      thisPacketType = handle_sensor_data(&(buffer[4]));
      break;
    case 0x15:
      //status info - unimplemented
      break;
    case 0x83:
      //firmware data - unimplemented
      break;
    }
    GPIO_WriteBit(GPIOD, xfer_pin, Bit_SET);
    GPIO_Init(GPIOD, &Xfer_input);
  }
  return thisPacketType;
}

packetType_t handle_sensor_data(unsigned char* data)
{
  packetType_t thisPacketType;
  if(data[SW_PAYLOAD_HDR_CONFIGMASK] & SW_DATA_XYZ && data[SW_PAYLOAD_HDR_SYSINFO] & SW_SYS_POSITION ){
    // Valid XYZ position
    x = data[SW_PAYLOAD_X+1] << 8 | data[SW_PAYLOAD_X];
    y = data[SW_PAYLOAD_Y+1] << 8 | data[SW_PAYLOAD_Y];
    z = data[SW_PAYLOAD_Z+1] << 8 | data[SW_PAYLOAD_Z];
    thisPacketType |= PACKET_XYZ;
  }
  
  if( data[SW_PAYLOAD_HDR_CONFIGMASK] & SW_DATA_GESTURE && data[SW_PAYLOAD_GESTURE] > 0){
    // Valid gesture
    last_gesture = data[SW_PAYLOAD_GESTURE];
    thisPacketType |= PACKET_GESTURE;
  }
  
  if ( data[SW_PAYLOAD_HDR_CONFIGMASK] & SW_DATA_TOUCH ){
    // Valid touch
    uint16_t touch_action = data[SW_PAYLOAD_TOUCH+1] << 8 | data[SW_PAYLOAD_TOUCH];
    uint16_t comp = 1 << 14;
    uint8_t x;
    for(x = 0; x < 16; x++){
      if( touch_action & comp ){
        last_touch = x;
        thisPacketType |= PACKET_TOUCH;
        return thisPacketType;
      }
      comp = comp >> 1;
    }
  }
  
  if( data[SW_PAYLOAD_HDR_CONFIGMASK] & SW_DATA_AIRWHEEL && data[SW_PAYLOAD_HDR_SYSINFO] & SW_SYS_AIRWHEEL ){
    
    double delta = (data[SW_PAYLOAD_AIRWHEEL] - lastrotation) / 32.0;
    
    if( (delta != 0) && (delta > -0.5) && (delta < 0.5)){
      //if( this->handle_airwheel != NULL ) this->handle_airwheel(delta * 360.0);  
      thisPacketType = PACKET_AIRWHEEL;
    }
    
    rotation += delta * 360.0;
    rotation %= 360;
    
    lastrotation = data[SW_PAYLOAD_AIRWHEEL];
  }
  return thisPacketType;
}


/* I2C1 init function */
static void MX_I2C1_Init(void)
{
  
  //hi2c1.Instance = I2C1;
  hi2c1.I2C_ClockSpeed = 200000;
  hi2c1.I2C_DutyCycle = I2C_DutyCycle_2;
  hi2c1.I2C_OwnAddress1 = 0;
  hi2c1.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  //hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  //hi2c1.Init.OwnAddress2 = 0;
  //hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  //hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  I2C_Init(I2C1, &hi2c1);
  /*if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }*/
}

unsigned char getGesture()
{
  return last_gesture;
}

int getRotation()
{
  return rotation;
}

void getXYZ(unsigned int *x_out, unsigned int *y_out, unsigned int *z_out)
{
  *x_out = x;
  *y_out = y;
  *z_out = z;
  return;
}