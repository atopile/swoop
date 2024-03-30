/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 19/03/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, the Easy Pro Mini,
//be sure to change the definitiosn to match your own setup. Some pins such as DIO2,
//DIO3, may not be in used by this sketch so they do not need to be connected and
//should be set to -1.

const int8_t SPI_MOSI = PA7;
const int8_t SPI_MISO = PA6;
const int8_t SPI_SCK = PA5;
const int8_t NSS = PA4;                          //select on LoRa device
const int8_t NRESET = PB4;                        //reset on LoRa device
const int8_t RFBUSY = PA11;                        //RF busy on LoRa device 
const int8_t DIO1 = PA10;                          //DIO1 on LoRa device, used for RX and TX done
const int8_t DIO2 = -1;                         //DIO2 on LoRa device, normally not used so set to -1
const int8_t DIO3 = -1;                         //DIO3 on LoRa device, normally not used so set to -1
// const int8_t LED1 = -1;                          //On board LED, logic high is on
const int8_t RX_EN = -1;                        //pin for RX enable, used on some SX1280 devices, set to -1 if not used
const int8_t TX_EN = -1;                        //pin for TX enable, used on some SX1280 devices, set to -1 if not used



const int8_t joystickX1 = A2;                   //analog pin for the joystick 1 X pot
const int8_t joystickY1 = A3;                   //analog pin for the joystick 1 Y pot
const int8_t SWITCH1 = 2;                       //switch on joystick, set to -1 if not used

const uint32_t TXIdentity = 123 ;               //define a transmitter number, the receiver must use the same number
                                                //range is 0 to 255


#define LORA_DEVICE DEVICE_SX1280               //we need to define the device we are using 


//*******  Setup LoRa Parameters Here ! ***************

//LoRa Modem Parameters
#define Frequency 2445000000                     //frequency of transmissions
#define Offset 0                                 //offset frequency for calibration purposes  
#define Bandwidth LORA_BW_0400                   //LoRa bandwidth
#define SpreadingFactor LORA_SF7                 //LoRa spreading factor
#define CodeRate LORA_CR_4_5                     //LoRa coding rate

const int8_t TXpower = 10;                      //LoRa transmit power in dBm

const uint16_t packet_delay = 1000;             //mS delay between packets