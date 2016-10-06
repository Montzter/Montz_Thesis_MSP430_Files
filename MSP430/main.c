#include <Montz_Thesis_Header.h>

int Timer_Counter_Limit = 0;
int System_Status = 0;
int sendStatusUpdates = 0;
int ImageSpacingDelay = 0;
int ImageDelayThreshold = 20*30;
char UART_PC_RX_char;
int UART_PC_char_received = 0;
char UART_PC_RX_header[8], UART_PC_TX_header[8];
char UART_PC_RX_DataPacket[26];
char UART_PC_TX_DataPacket[256];//Make array the size of data packet
int transmit_UART_PC;


void main(void)
{
  int UART_PC_RX_MSG_size = 0;
  char PC_Status_stand_in;
  int i;
  int System_ID = 1;
  unsigned int UART_PC_RX_header_CRC;
  UART_PC_TX_header[0] = 0x6E;
  UART_PC_TX_header[2] = System_ID;
  transmit_UART_PC = 0;

//Stop watchdog timer
  WDTCTL = WDTPW | WDTHOLD;

//Increment voltage
  IncrementVcore();
  IncrementVcore();

//Set MCLK and SMCLK to XT1
  UCSCTL1 |= DCORSEL_5;
  UCSCTL2 = 512;
  
//Crystal Oscillator settings
  UCSCTL6 &= ~XCAP_1;

//output MCLK to P11.1, pin 85
  //outputMCLKP11_1;

//output SMCLK to P11.2, pin 86
  //outputSMCLKP11_2;

//Timer A0 setup
  TA0CTL |= TASSEL__SMCLK + ID__8 + MC__STOP;
  TA0CCTL1 |= OUTMOD_6;
  TA0EX0 = TAIDEX_7;
  setPWM_TA01_20msPeriod;
  //setPWM_TA01_1_25ms;
  //setPWM_TA01_1_50ms;
  //setPWM_TA01_1_75ms;
  //PWM_on;

//Timer A CCR1 to P8.1, pin 58
  outputTA0_1;

//Timer A1 setup
  TA1CTL |= TASSEL__SMCLK + ID__8 + MC__STOP;// + TAIE;
  TA1CCTL0 |= CCIE;
  TA1EX0 = TAIDEX_7;
  TA1CCR0 = 13107;
  TIMER_250msISR;
  //TIMER_on;

//UART UCA3
  UCA3CTL1 |= UCSWRST;
  UCA3CTL1 |= UCSSEL__SMCLK;
  UCA3BR0 = 9;
  UCA3MCTL |= UCOS16 + UCBRF_2;//1 + UCBRS_6;
  P10SEL |= BIT4 + BIT5;
  UCA3CTL1 &= ~UCSWRST;
  UCA3IE |= UCTXIE + UCRXIE;

//Test Timer A1
  //outputPin1_7;

//Initialize everything to off
  System_Status = 0;
  FPGA_off;
  SERVO_off;
  ThreeV3_Switch_off;
  QUARK_off;
  outputPins6_4_7;
  _EINT();

//Main Loop
  while(1)
  {
    LPM0;
    if(UART_PC_char_received == 1)//handle new char from PC;
    {
      UART_PC_char_received = 0;

      if(UART_PC_RX_MSG_size == 0 && UART_PC_RX_char == 0x6E)//detect first character(Process Code) '0x6E'
      {
        UART_PC_RX_header[0] = UART_PC_RX_char;
        UART_PC_RX_header_CRC = 0;
        UART_PC_RX_MSG_size++;
      }
      else if(UART_PC_RX_MSG_size == 1)//detect second character(Status)
      {
        UART_PC_RX_header[1] = UART_PC_RX_char;
        UART_PC_RX_MSG_size++;
      }
      else if(UART_PC_RX_MSG_size == 2)//detect third character(Reserved)
      {
        UART_PC_RX_header[2] = UART_PC_RX_char;
        UART_PC_RX_MSG_size++;
      }
      else if(UART_PC_RX_MSG_size == 3)//detect fourth character(Function)
      {
        UART_PC_RX_header[3] = UART_PC_RX_char;
        UART_PC_RX_MSG_size++;
      }
      else if(UART_PC_RX_MSG_size == 4)//detect fifth character(ByteCount_MSB)
      {
        UART_PC_RX_header[4] = UART_PC_RX_char;
        UART_PC_RX_MSG_size++;
      }
      else if(UART_PC_RX_MSG_size == 5)//detect sixth character(ByteCount_LSB)
      {
        UART_PC_RX_header[5] = UART_PC_RX_char;
        UART_PC_RX_MSG_size++;
      }
      else if(UART_PC_RX_MSG_size == 6)//detect sixth character(CRC_MSB)
      {
        UART_PC_RX_header[6] = UART_PC_RX_char;
        UART_PC_RX_header_CRC = (unsigned int)UART_PC_RX_header[6] << 8;
        UART_PC_RX_MSG_size++;
      }
      else if(UART_PC_RX_MSG_size == 7)//detect sixth character(CRC_LSB)
      {
        UART_PC_RX_header[7] = UART_PC_RX_char;
        UART_PC_RX_header_CRC += (unsigned int)UART_PC_RX_header[7];
        UART_PC_RX_MSG_size++;
      }

      if(UART_PC_RX_MSG_size == 8)//verify CRC
      {
        //Initialize CRC module
          CRCINIRES = 0x0000;
        //Run UART_PC_RX_header through CRC module
          for(i = 0; i < 6; i++)
          {
            CRCDIRB_L = UART_PC_RX_header[i];
          }

        //Compare CRC module result with crc of header        
        if(UART_PC_RX_header_CRC == CRCINIRES)
        {
          //CRC Match
          switch(UART_PC_RX_header[3])//Based on Function byte...
          {
          case 0://Abort/Shutdown message
          {
            System_Status = 0;//reset system state;
            UART_PC_SEND_ACK();
            PWM_off;//Turn off PWM signal
            TIMER_off;//Turn off timer
            QUARK_off;//Turn off quark
            ThreeV3_Switch_off;//Turn off S.FLASH
            SERVO_off;//Turn off SERVO
            FPGA_off;//Turn off FPGA
            UART_PC_char_received = 0;
            UART_PC_RX_MSG_size = 0;
            break;
          }
          case 1://ACK message
          {

            break;
          }
          case 2://Detect System message
          {
            if(UART_PC_RX_header[1] == 0x00)
            {
              sendStatusUpdates = 0;
            }
            else if(UART_PC_RX_header[1] == 0xFF)
            {
              sendStatusUpdates = 1;
            }
            UART_PC_SEND_STATUS_UPDATE();
            break;
          }
          case 3://Begin Process message
          {
            ImageSpacingDelay = 20 * UART_PC_RX_header[1];
            
            //Setup System_State 1; Turn on Quark and S.FLASH, wait 4 seconds
            QUARK_on;
            ThreeV3_Switch_on; //Turn on Quark and S.FLASH
            PWM_off;
            TIMER_4sISR;//wait 4 sec
            System_Status = 1;
            TIMER_on;

            UART_PC_SEND_ACK();
            break;
          }
          case 4://Begin Transmitting Data message
          {
            //Setup System State 35; Turn on MSP430 and S.Flash, Load data from SFLASH
            QUARK_off;//Double check that Quark is off
            ThreeV3_Switch_on;//Turn on S.Flash
            TIMER_250msISR;//Stand in timer, should wait until data is loaded to msp430
            System_Status = 35;
            TIMER_on;
            UART_PC_SEND_ACK();          
            break;
          }
          case 5://Orientation Data message
          {
            UART_PC_SEND_ACK();
            break;
          }
          case 0xFF://NACK message
          {

            break;
          }
          }

          //UCA3TXBUF = 'U';
        }
        else
        {
          UCA3TXBUF = 'X';
        }

        UART_PC_RX_MSG_size = 0;
      }
    }
    //UCA3TXBUF = UCA3RXBUF;
  }
}

void UART_PC_ERROR_SHUTDOWN(void)
{
  int i = 0;

  //UART_PC_TX_header[0] = 0x6E;//Process Code = 0x6E; removed due to constant value
  UART_PC_TX_header[1] = 0x00;//Status Byte = 0;
  //UART_PC_TX_header[2] = 0x01;//Reserved Byte = 1; removed due to constant value
  UART_PC_TX_header[3] = 0x00;//Function Byte = 0;
  UART_PC_TX_header[4] = 0x00;//ByteCountMSB Byte = 0;
  UART_PC_TX_header[5] = 0x00;//ByteCountLSB Byte = 0;
  UART_PC_TX_header[6] = 0xA9;//CRC_MSB Byte = 0;
  UART_PC_TX_header[7] = 0x0F;//CRC_LSB Byte = 0;


  UCA3TXBUF = UART_PC_TX_header[0];
  for(i = 1; i < 8; i++)
  {
    while(!(UCA3IFG & UCTXIFG));
    UCA3TXBUF = UART_PC_TX_header[i];
  }
}

void UART_PC_SEND_ACK(void)
{
  int i = 0;

  //UART_PC_TX_header[0] = 0x6E;//Process Code = 0x6E; removed due to constant value
  UART_PC_TX_header[1] = System_Status;//Status Byte
  //UART_PC_TX_header[2] = 0x01;//Reserved Byte = 1; removed due to constant value
  UART_PC_TX_header[3] = 0x01;//Function Byte = 0;
  UART_PC_TX_header[4] = 0x00;//ByteCountMSB Byte = 0;
  UART_PC_TX_header[5] = 0x00;//ByteCountLSB Byte = 0;
  //UART_PC_TX_header[6] = 0xA9;//CRC_MSB Byte = 0;
  //UART_PC_TX_header[7] = 0x0F;//CRC_LSB Byte = 0;

  CRCINIRES = 0x0000;
  for(i = 0; i < 6; i++)
  {
    CRCDIRB_L = UART_PC_TX_header[i];
  }
  UART_PC_TX_header[6] = CRCINIRES_H;
  UART_PC_TX_header[7] = CRCINIRES_L;

  UCA3TXBUF = UART_PC_TX_header[0];
  for(i = 1; i < 8; i++)
  {
    while(!(UCA3IFG & UCTXIFG));
    UCA3TXBUF = UART_PC_TX_header[i];
  }
}

void UART_PC_SEND_STATUS_UPDATE(void)
{
  int i = 0;

  //UART_PC_TX_header[0] = 0x6E;//Process Code = 0x6E; removed due to constant value
  UART_PC_TX_header[1] = System_Status;//Status Byte
  //UART_PC_TX_header[2] = 0x01;//Reserved Byte = 1; removed due to constant value
  UART_PC_TX_header[3] = 0x02;//Function Byte = 0;
  UART_PC_TX_header[4] = 0x00;//ByteCountMSB Byte = 0;
  UART_PC_TX_header[5] = 0x00;//ByteCountLSB Byte = 0;
  //UART_PC_TX_header[6] = 0xA9;//CRC_MSB Byte = 0;
  //UART_PC_TX_header[7] = 0x0F;//CRC_LSB Byte = 0;

  CRCINIRES = 0x0000;
  for(i = 0; i < 6; i++)
  {
    CRCDIRB_L = UART_PC_TX_header[i];
  }
  UART_PC_TX_header[6] = CRCINIRES_H;
  UART_PC_TX_header[7] = CRCINIRES_L;

  UCA3TXBUF = UART_PC_TX_header[0];
  for(i = 1; i < 8; i++)
  {
    while(!(UCA3IFG & UCTXIFG));
    UCA3TXBUF = UART_PC_TX_header[i];
  }
}

void UART_PC_SEND_REQUEST_ORIENTATION_DATA(void)
{
  int i = 0;

  //UART_PC_TX_header[0] = 0x6E;//Process Code = 0x6E; removed due to constant value
  UART_PC_TX_header[1] = System_Status;//Status Byte
  //UART_PC_TX_header[2] = 0x01;//Reserved Byte = 1; removed due to constant value
  UART_PC_TX_header[3] = 0x03;//Function Byte = 0;
  UART_PC_TX_header[4] = 0x00;//ByteCountMSB Byte = 0;
  UART_PC_TX_header[5] = 0x00;//ByteCountLSB Byte = 0;
  //UART_PC_TX_header[6] = 0xA9;//CRC_MSB Byte = 0;
  //UART_PC_TX_header[7] = 0x0F;//CRC_LSB Byte = 0;

  CRCINIRES = 0x0000;
  for(i = 0; i < 6; i++)
  {
    CRCDIRB_L = UART_PC_TX_header[i];
  }
  UART_PC_TX_header[6] = CRCINIRES_H;
  UART_PC_TX_header[7] = CRCINIRES_L;

  UCA3TXBUF = UART_PC_TX_header[0];
  for(i = 1; i < 8; i++)
  {
    while(!(UCA3IFG & UCTXIFG));
    UCA3TXBUF = UART_PC_TX_header[i];
  }
}

void UART_PC_SEND_DATA_TRANSFER(void)
{

}

void USCI_A3_isr(void)__interrupt[USCI_A3_VECTOR]
{
int i;

  switch(UCA3IV)
  {
    case 0x02: //UCRXIFG flag
    {
      UART_PC_RX_char = UCA3RXBUF;
      UART_PC_char_received = 1;
      LPM0_EXIT;
      break;
    }
    case 0x04: //UCTXIFG flag
    {

//     if(transmit_UART_PC = 1)
//     {
//      for(i = 1; i < 8; i++)
//      {
//        while(!(UCA3IFG&UCTXIFG));
//        UCA3TXBUF = UART_PC_TX_header[i];
//      }
//     }

    break;
    }
  }
}

void TimerA1_isr(void)__interrupt[TIMER1_A0_VECTOR]
{
static int Timer_Counter = 0;
  switch(TA1IV)
  {
  case 0: Timer_Counter++;break;
  case 2: break;
  case 4: break;
  case 6: break;
  case 8: break;
  case 10: break;
  case 12: break;
  case 14: break;
  default: break;
  }

  if(Timer_Counter >= Timer_Counter_Limit)
  {
    Timer_Counter = 0;
    //P1OUT ^= BIT7;
    switch(System_Status)
    {
      case 0:
      {
        //Setup System_State 1; Turn on Quark and S.FLASH, wait 4 seconds
        QUARK_on;
        ThreeV3_Switch_on; //Turn on Quark and S.FLASH
        PWM_off;
        TIMER_4sISR;//wait 4 sec
        System_Status++;
        break;
      }
      case 1:
      {
        //Setup System_State 2; Turn on FPGA, wait 0.25 seconds
        FPGA_on;//Turn on FPGA
        TIMER_250msISR;//wait 0.25 sec
        System_Status++;
        break;
      }
      case 2:
      {
        //Setup System State 3; Turn on Servo, Rotate servo to Black Body, wait 0.75 seconds
        SERVO_on;//Turn on Servo
        PWM_on;
        setPWM_BlackBody;//rotate to black body
        TIMER_750msISR;//wait 0.75 sec
        System_Status++;
        break;
      }
      case 3:
      {
        //Setup System State 4; Send DO FFC command to QUARK, wait 0.4 seconds
        PWM_off;
        TIMER_400msISR;//wait 0.4 seconds
        System_Status++;
        break;
      }
      case 4:
      {
        //Setup System State 5; Turn Servo to Filter A, wait 0.75 seconds
        PWM_on;
        setPWM_FilterA;//rotate to filter A
        TIMER_750msISR;//wait 0.75 seconds
        System_Status++;
        break;
      }
      case 5:
      {
        //Setup System State 6; Capture Image 1A, Start imgTimer;
        PWM_off;
        TIMER_250msISR;//Stand in timer. Wait for FPGA done signal for state exit trigger.
        System_Status++;
        break;
      }
      case 6:
      {
        //Setup System State 7;Turn servo to Black Body; wait 0.75 seconds;
        PWM_on;
        setPWM_BlackBody;//rotate to black body
        TIMER_750msISR;//wait 0.75 seconds
        System_Status++;
        break;
      }
      case 7:
      {
        //Setup System State 8; Send DO FFC command to QUARK, wait 0.4 seconds
        PWM_off;
        TIMER_400msISR;//wait 0.4 seconds
        System_Status++;
        break;
      }
      case 8:
      {
        //Setup System State 9; Turn servo to Filter B, wait 0.75 seconds
        PWM_on;
        setPWM_FilterB;//rotate to filter B
        TIMER_750msISR;//wait 0.75 seconds
        System_Status++;
        break;
      }
      case 9:
      {
        //Setup System State 10; Capture Image 1B
        PWM_off;
        TIMER_250msISR;//Stand in timer. Wait for FPGA done signal for state exit trigger.
        System_Status++;
        break;
      }
      case 10:
      {
        //Setup System State 11; Turn servo to black body, wait 0.75 seconds;
        PWM_on;
        setPWM_BlackBody;//rotate to black body;
        TIMER_750msISR;//wait 0.75 seconds;
        System_Status++;
        break;
      }
      case 11:
      {
        //Setup System State 12; Shutdown Quark,S.Flash,SERVO,FPGA, Wait for imgTimer to run out
        QUARK_off;//Turn off quark
        ThreeV3_Switch_off;//Turn off S.FLASH
        SERVO_off;//Turn off SERVO
        FPGA_off;//Turn off FPGA
        PWM_off;
        TIMER_250msISR;//Stand in timer. Wait for imgTimer to run out.
        System_Status++;
        break;
      }
      case 12:
      {
        //Setup System State 13; Turn on Quark and S.Flash, wait 4 seconds
        ThreeV3_Switch_on;//Turn on S.Flash
        QUARK_on;//Turn on QUARK
        TIMER_4sISR;//wait 4 seconds
        System_Status++;
        break;
      }
      case 13:
      {
        //Setup System State 14; Turn on FPGA, wait 1 second
        FPGA_on;//Turn on FPGA
        TIMER_1sISR;//wait 1 second
        System_Status++;
        break;
      }
      case 14:
      {
        //Setup System State 15; Send DO FFC command to QUARK, wait 0.4 seconds
        TIMER_400msISR;//wait 0.4 seconds
        System_Status++;
        break;
      }
      case 15:
      {
        //Setup System State 16; Turn servo to Filter A, wait 0.75 seconds;
        SERVO_on;
        PWM_on;
        setPWM_FilterA;//turn servo to filter A
        TIMER_750msISR;//wait 0.75 seconds
        System_Status++;
        break;
      }
      case 16:
      {
        //Setup System state 17; capture Image 2A
        PWM_off;
        TIMER_250msISR;//Stand in timer. Wait for FPGA done signal for state exit trigger.
        System_Status++;
        break;
      }
      case 17:
      {
        //Setup System state 18; Turn Servo to Black Body, wait 0.75 seconds
        PWM_on;
        setPWM_BlackBody;//turn servo to black body
        TIMER_750msISR;//wait 0.75 seconds
        System_Status++;
        break;
      }
      case 18:
      {
        //Setup System state 19; Send DO FFC command to QUARK, wait 0.4 seconds
        PWM_off;
        TIMER_400msISR;//wait 0.4 seconds
        System_Status++;
        break;
      }
      case 19:
      {
        //Setup System state 20; Turn servo to Filter B, wait 0.75 seconds
        PWM_on;
        setPWM_FilterB;//turn servo to filter B
        TIMER_750msISR;//wait 0.75 seconds
        System_Status++;
        break;
      }
      case 20:
      {
        //Setup System state 21; Capture Image 2B
        PWM_off;
        TIMER_250msISR;//Stand in timer. Wait for FPGA done signal for state exit trigger.
        System_Status++;
        break;
      }
      case 21:
      {
        //Setup System state 22; Turn servo to Black Body, wait 0.75 seconds
        PWM_on;
        setPWM_BlackBody;//turn servo to black body
        TIMER_750msISR;//wait 0.75 seconds
        System_Status++;
        break;
      }
      case 22:
      {
        //Setup System state 23; Shutdown Quark,S.Flash,SERVO,FPGA, Wait for imgTimer to run out
        QUARK_off;//Turn off quark
        ThreeV3_Switch_off;//Turn off S.FLASH
        SERVO_off;//Turn off SERVO
        FPGA_off;//Turn off FPGA
        PWM_off;
        TIMER_250msISR;//Stand in timer. Wait for imgTimer to run out.
        System_Status++;
        break;
      }
      case 23:
      {
        //Setup System state 24; Turn on Quark and S.Flash, wait 4 seconds
        ThreeV3_Switch_on;//Turn on S.Flash
        QUARK_on;//Turn on QUARK
        TIMER_4sISR;  //wait 4 seconds
        System_Status++;
        break;
      }
      case 24:
      {
        //Setup System state 25; Turn on FPGA, wait 1 second
        FPGA_on;
        TIMER_1sISR;//wait 1 second
        System_Status++;
        break;
      }
      case 25:
      {
        //Setup System State 26; Send DO FFC command to QUARK, wait 0.4 seconds
        TIMER_400msISR;//wait 0.4 seconds
        System_Status++;
        break;
      }
      case 26:
      {
        //Setup System State 27; Turn servo to Filter A, wait 0.75 seconds
        SERVO_on;
        PWM_on;
        setPWM_FilterA;//turn servo to filter A
        TIMER_750msISR;//wait 0.75 seconds
        System_Status++;
        break;
      }
      case 27:
      {
        //Setup System State 28; Capture image 3A;
        PWM_off;
        TIMER_250msISR;//Stand in timer. Wait for FPGA done signal for state exit trigger.
        System_Status++;
        break;
      }
      case 28:
      {
        //Setup System State 29; Turn Servo to Black Body, wait 0.75 seconds
        PWM_on;
        setPWM_BlackBody;//turn to black body
        TIMER_750msISR;//wait 0.75 seconds
        System_Status++;
        break;
      }
      case 29:
      {
        //Setup System State 30; Send DO FFC command to QUARK, wait 0.4 seconds
        PWM_off;
        TIMER_400msISR;//wait 0.4 seconds
        System_Status++;
        break;
      }
      case 30:
      {
        //Setup System State 31; Turn servo to Filter B, wait 0.75 seconds
        PWM_on;
        setPWM_FilterB;//turn servo to filter B
        TIMER_750msISR;//wait 0.75 seconds
        System_Status++;
        break;
      }
      case 31:
      {
        //Setup System State 32; Capture image 3B
        PWM_off;
        TIMER_250msISR;//Stand in timer. Wait for FPGA done signal for state exit trigger.
        System_Status++;
        break;
      }
      case 32:
      {
        //Setup System State 33; Turn Servo to Black Body, wait 0.75 seconds
        PWM_on;
        setPWM_BlackBody;//turn servo to black body
        TIMER_750msISR;//wait 0.75 seconds
        System_Status++;
        break;
      }
      case 33:
      {
        //Setup System State 34; Shutdown Quark,S.Flash,SERVO,FPGA
        QUARK_off;//Turn off quark
        ThreeV3_Switch_off;//Turn off S.FLASH
        SERVO_off;//Turn off SERVO
        FPGA_off;//Turn off FPGA
        PWM_off;
        TIMER_off;
        System_Status++;
        break;
      }
      //***************states 35 - 38/0 should be handled in the main operation loop, not here in the timer.
      case 34:
      {
        //Setup System State 35; Turn on MSP430 and S.Flash, Load data from SFLASH
        QUARK_off;//Double check that Quark is off
        ThreeV3_Switch_on;//Turn on S.Flash
        TIMER_250msISR;//Stand in timer, should wait until data is loaded to msp430
        System_Status++;
        break;
      }
      case 35:
      {
        //Setup System State 36; Transmit data to PC. Wait for ACK or Timeout
        TIMER_250msISR;//stand in timeout clock
        System_Status++;
        break;
      }
      case 36:
      {
        //Setup System State 37; Load more data from S.FLASH, repeat until done
        TIMER_250msISR;
        System_Status++;
        break;
      }
      case 37:
      {
        //Setup System State 0; Turn off S.Flash
        QUARK_off;//Turn off quark
        ThreeV3_Switch_off;//Turn off S.FLASH
        SERVO_off;//Turn off SERVO
        FPGA_off;//Turn off FPGA
        TIMER_off;
        //TIMER_250msISR;
        System_Status = 0;
        break;
      }
      default: break;
    }
    if(sendStatusUpdates)
      UART_PC_SEND_STATUS_UPDATE();

  }
}

