#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define UART_ID uart0
#define BAUD_RATE 100000      //115200
#define DATA_BITS 8
#define STOP_BITS 2             //1
#define PARITY    UART_PARITY_EVEN
#define DUTYMIN 1330
#define DUTYMAX 1520
#define CH1MAX 1761
#define CH1MIN 281
#define CH2MAX 1727
#define CH2MIN 315
#define CH3MAX 1728
#define CH3MIN 323
#define CH4MAX 1753
#define CH4MIN 310
#define CH5MAX 1904
#define CH5MIN 144
#define CH6MAX 1904
#define CH6MIN 144
//０番と1番ピンに接続
#define UART_TX_PIN 0
#define UART_RX_PIN 1

//グローバル変数
static int chars_rxed = 0;
static int data_num=0;
uint8_t sbus_data[25];
uint8_t ch =0;
uint slice_num[2];
uint16_t Olddata[6];
uint16_t Chdata[6];
float Data1=0.5,Data2=0.5,Data3=0,Data4=0.5,Data5=0,Data6=0;
float Duty_rr,Duty_rl,Duty_fl,Duty_fr;
//関数の宣言
uint8_t serial_settei(void);
uint8_t pwm_settei();
void on_uart_rx();

//シリアル設定
uint8_t serial_settei(void){
    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, 2400);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set dat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asheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    int actual = uart_set_baudrate(UART_ID, BAUD_RATE);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);
}
uint8_t pwm_settei(){
    //pwmの設定
    // Tell GPIO 0 and 1 they are allocated to the PWM
    gpio_set_function(2, GPIO_FUNC_PWM);
    gpio_set_function(3, GPIO_FUNC_PWM);
   
    gpio_set_function(4, GPIO_FUNC_PWM);
    gpio_set_function(5, GPIO_FUNC_PWM);
    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    slice_num[0] = pwm_gpio_to_slice_num(3);
    slice_num[1] = pwm_gpio_to_slice_num(4);

    // Set period
    pwm_set_wrap(slice_num[0], 3124);
    pwm_set_clkdiv(slice_num[0], 100.0);
    pwm_set_wrap(slice_num[1], 3124);
    pwm_set_clkdiv(slice_num[1], 100.0);
    // Set channel A output high for one cycle before dropping
    pwm_set_chan_level(slice_num[0], PWM_CHAN_A, DUTYMAX);
    pwm_set_chan_level(slice_num[0], PWM_CHAN_B, DUTYMAX);
    pwm_set_chan_level(slice_num[1], PWM_CHAN_A, DUTYMAX);
    pwm_set_chan_level(slice_num[1], PWM_CHAN_B, DUTYMAX);
    // Set the PWM running
    pwm_set_enabled(slice_num[0], true);
    pwm_set_enabled(slice_num[1], true);
    /// \end::setup_pwm[]
    sleep_ms(2000);
    pwm_set_chan_level(slice_num[0], PWM_CHAN_A, DUTYMIN);
    pwm_set_chan_level(slice_num[0], PWM_CHAN_B, DUTYMIN);
    pwm_set_chan_level(slice_num[1], PWM_CHAN_A, DUTYMIN);
    pwm_set_chan_level(slice_num[1], PWM_CHAN_B, DUTYMIN);
}

void on_uart_rx() {
    short data;
    while (uart_is_readable(UART_ID)) {
        ch = uart_getc(UART_ID);
        if(ch==0x0f&&chars_rxed==00){
            sbus_data[chars_rxed]=ch;
            //printf("%02X ",ch);
            chars_rxed++;
        }
        else if(chars_rxed>0){
            sbus_data[chars_rxed]=ch;
            //printf("%02X ",ch);
            chars_rxed++;            
        }


        // Can we send it back?
        //if (uart_is_writable(UART_ID)) {
        //    // Change it slightly first!
            //ch++;
        //    uart_putc(UART_ID, ch);
        //}
        
        switch(chars_rxed){
            case 3:
                Olddata[0]=(sbus_data[1]|(sbus_data[2]<<8)&0x07ff);
		Data1=(float)(Olddata[0]-CH1MIN)/(CH1MAX-CH1MIN);
                //printf("%04f ",Data1);
                break;
            case 4:
                Olddata[1]=(sbus_data[3]<<5|sbus_data[2]>>3)&0x07ff;	
		Data2=(float)(Olddata[1]-CH2MIN)/(CH2MAX-CH2MIN);
                //printf("%04f ",Data2);
                break;
            case 6:
                Olddata[2]=(sbus_data[3]>>6|sbus_data[4]<<2|sbus_data[5]<<10)&0x07ff;
		Data3=(float)(Olddata[2]-CH3MIN)/(CH3MAX-CH3MIN);
                //printf("%04f ",Data3);
                break;
            case 7:
                Olddata[3]=(sbus_data[6]<<7|sbus_data[5]>>1)&0x07ff;
		Data4=(float)(Olddata[3]-CH4MIN)/(CH4MAX-CH4MIN);
                //printf("%04f ",Data4);
                break;
            case 8:
                Olddata[4]=(sbus_data[7]<<4|sbus_data[6]>>4)&0x07ff;
		Data5=(float)(Olddata[4]-CH5MIN)/(CH5MAX-CH5MIN);
                //printf("%04f ",Data5);
                break;
            case 10:
                Olddata[5]=(sbus_data[7]>>7|sbus_data[8]<<1|sbus_data[9]<<9)&0x07ff;
		Data6=(float)(Olddata[5]-CH6MIN)/(CH6MAX-CH6MIN);
                //printf("%04f ",Data6);
                break;
                
        }

        if(chars_rxed==25){
            printf("\n");
            chars_rxed=0;
        }
        
    }
}

	
int main(){  
    float duty_rr,duty_rl,duty_fl,duty_fr;
    float seigyo=0.5;  
    stdio_init_all(); //シリアル通信の設定
    pwm_settei();
    serial_settei();
  //  elevator();
    sleep_ms(2000);
    
    while(true){	
	Duty_fr=Data3+((Data2-.5)-(Data4-.5)-(Data1-.5))*seigyo;
	Duty_fl=Data3+(-1.5+Data2+Data4+Data1)*seigyo;
	Duty_rr=Data3+(-(Data2-.5)-(Data4-.5)+(Data1-.5))*seigyo;
	Duty_rl=Data3+(-(Data2-.5)+(Data4-.5)-(Data1-.5))*seigyo;

	tight_loop_contents();
      	duty_rr=(float)(DUTYMAX-DUTYMIN)*Duty_rr+DUTYMIN;
      	duty_fr=(float)(DUTYMAX-DUTYMIN)*Duty_fr+DUTYMIN;
      	duty_rl=(float)(DUTYMAX-DUTYMIN)*Duty_rl+DUTYMIN;
      	duty_fl=(float)(DUTYMAX-DUTYMIN)*Duty_fl+DUTYMIN;
      	if (duty_rr>DUTYMAX)duty_rr=DUTYMAX;
      	if (duty_rr<DUTYMIN)duty_rr=DUTYMIN;
      	if (duty_fr>DUTYMAX)duty_fr=DUTYMAX;
      	if (duty_fr>DUTYMAX)duty_fr=DUTYMAX;
      	if (duty_rl>DUTYMAX)duty_rl=DUTYMAX;
      	if (duty_rl<DUTYMIN)duty_rl=DUTYMIN;
      	if (duty_fl<DUTYMIN)duty_fl=DUTYMIN;
      	if (duty_fl<DUTYMIN)duty_fl=DUTYMIN;
      	pwm_set_chan_level(slice_num[0], PWM_CHAN_A, duty_rr);
      	pwm_set_chan_level(slice_num[0], PWM_CHAN_B, duty_fr);
      	pwm_set_chan_level(slice_num[1], PWM_CHAN_A, duty_rl);
      	pwm_set_chan_level(slice_num[1], PWM_CHAN_B, duty_fl);
      	printf("%04f %04f %04f %04f %04f %04f %04f %04f %04f %04f \n",Data1,Data2,Data3,Data4,Data5,Data6,duty_rr,duty_rl,duty_fl,duty_fr);  
      	sleep_ms(10);
   }

}

// RX interrupt handler


