#include <msp430.h>

//Output pins
#define outputMCLKP11_1   P11DIR |= BIT1; P11SEL |= BIT1; //Pin 85
#define outputSMCLKP11_2  P11DIR |= BIT2; P11SEL |= BIT2; //Pin 86
#define outputTA0_1       P8DIR |= BIT1; P8SEL |= BIT1;   //Pin 58
#define outputPin1_7      P1DIR |= BIT7; P1SEL &= ~BIT7;  //Pin 24
#define outputPins6_4_7   P6DIR |= BIT4 + BIT5 + BIT6 + BIT7; P6SEL &= ~(BIT4 + BIT5 + BIT6 + BIT7); //Pins 1 -> 4

//Component control pins, P6.4 -> P6.7
//Control 12V Relay, P6.4, pin 1
#define FPGA_on       P6OUT |= BIT4;
#define FPGA_off      P6OUT &= ~BIT4;
//Control MOSFET supplying 5V to Servo, P6.5, pin 2
#define SERVO_on     P6OUT |= BIT5;
#define SERVO_off      P6OUT &= ~BIT5;
//Control MOSFET supplying 3.3V to QUARK and S.Flash, P6.6, pin 3
#define ThreeV3_Switch_on      P6OUT |= BIT6;
#define ThreeV3_Switch_off     P6OUT &= ~BIT6;
//Control QUARK_SHDN pin;, P6.7, pin 4
#define QUARK_on     P6OUT |= BIT7;
#define QUARK_off    P6OUT &= ~BIT7;

//Timer setup
#define PWM_on                    TA0CTL &= ~MC__UPDOWN; TA0CTL |= MC__UP;
#define PWM_off                   TA0CTL &= ~MC__UPDOWN;
#define setPWM_TA01_20msPeriod    TA0CCR0 = 5245;//5245;
#define setPWM_TA01_1_25ms        TA0CCR1 = 335;//328;
#define setPWM_TA01_1_50ms        TA0CCR1 = 395;//393;
#define setPWM_TA01_1_75ms        TA0CCR1 = 459;
#define setPWM_BlackBody          TA0CCR1 = 335;
#define setPWM_FilterA            TA0CCR1 = 395;
#define setPWM_FilterB            TA0CCR1 = 459;

#define TIMER_on            TA1CTL &= ~MC__UPDOWN; TA1CTL |= MC__UP;
#define TIMER_off           TA1CTL &= ~MC__UPDOWN;
#define TIMER_50msPeriod    TA1CCR0 = 13107;

#define TIMER_250msISR      Timer_Counter_Limit = 5;
#define TIMER_400msISR      Timer_Counter_Limit = 8;
#define TIMER_750msISR      Timer_Counter_Limit = 15;
#define TIMER_1sISR         Timer_Counter_Limit = 20;
#define TIMER_4sISR         Timer_Counter_Limit = 80;



int IncrementVcore(void);
void UART_PC_ERROR_SHUTDOWN(void);
void UART_PC_SEND_ACK(void);
void UART_PC_SEND_STATUS_UPDATE(void);
void UART_PC_SEND_REQUEST_ORIENTATION_DATA(void);
void UART_PC_SEND_DATA_TRANSFER(void);

