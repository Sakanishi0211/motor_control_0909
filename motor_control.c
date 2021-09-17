#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "lsm9ds1_reg.h"

#define UART_ID uart0
#define BAUD_RATE 100000      //115200
#define DATA_BITS 8
#define STOP_BITS 2             //1
#define PARITY    UART_PARITY_EVEN
#define DUTYMIN 1330
#define DUTYMAX 1520
#define CH1MAX 1748
#define CH1MIN 332
#define CH2MAX 1720
#define CH2MIN 324
#define CH3MAX 1724
#define CH3MIN 323
#define CH4MAX 1715
#define CH4MIN 340
#define CH5MAX 1904
#define CH5MIN 144
#define CH6MAX 1904
#define CH6MIN 144
//０番と1番ピンに接続
#define UART_TX_PIN 0
#define UART_RX_PIN 1
/* Private macro -------------------------------------------------------------*/
#define BOOT_TIME 20 //ms
#define PIN_CSAG  6
#define PIN_MISO  7
#define PIN_CSM   8
#define PIN_SCK   9
#define PIN_MOSI  10

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
nclude <string.h>

/* Define communication interface */
#define SENSOR_BUS spi0


typedef struct {
  spi_inst_t   *hbus;
  uint16_t cs_pin;
} sensbus_t;

/* Private variables ---------------------------------------------------------*/
static sensbus_t imu_bus = {SENSOR_BUS,
                            PIN_CSAG
                           };
static sensbus_t mag_bus = {SENSOR_BUS,
                            PIN_CSM
                           };

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_magnetic_field[3];
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_field_mgauss[3];
static lsm9ds1_id_t whoamI;
static lsm9ds1_status_t reg;
static uint8_t rst;
static uint8_t tx_buffer[1000];
float Acceleration_mg[3];
float Angular_rate_mdps[3];
float Magnetic_field_mgauss[3];                     
stmdev_ctx_t dev_ctx_imu;
stmdev_ctx_t dev_ctx_mag;
/* Private functions ---------------------------------------------------------*/
  /*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */

static int32_t platform_write_imu(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len);
static int32_t platform_read_imu(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len);
static int32_t platform_write_mag(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len);
static int32_t platform_read_mag(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/*付け加えた*/
static inline void cs_select(uint16_t cs_pin);
static inline void cs_deselect(uint16_t cs_pin);

uint16_t nine_dof_settei(){
  /* Initialize inertial sensors (IMU) driver interface */
  dev_ctx_imu.write_reg = platform_write_imu;
  dev_ctx_imu.read_reg = platform_read_imu;
  dev_ctx_imu.handle = (void *)&imu_bus;
  /* Initialize magnetic sensors driver interface */
  dev_ctx_mag.write_reg = platform_write_mag;
  dev_ctx_mag.read_reg = platform_read_mag;
  dev_ctx_mag.handle = (void *)&mag_bus;
  /* Initialize platform specific hardware */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);

  if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID) {
    while (1) {
      /* manage here device not found */
      printf("Device not found !\n");
      sleep_ms(1000);
    }
  }

  /* Restore default configuration */
  lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

  do {
    lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu,
                                PROPERTY_ENABLE);
  /* Set full scale */
  lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
  lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
  lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);
  /* Configure filtering chain - See datasheet for filtering chain details */
  /* Accelerometer filtering chain */
  lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
  lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu,
                                     LSM9DS1_LP_ODR_DIV_50);
  lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
  /* Gyroscope filtering chain */
  lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu,
                                     LSM9DS1_LP_ULTRA_LIGHT);
  lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
  lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu,
                                 LSM9DS1_LPF1_HPF_LPF2_OUT);
  /* Set Output Data Rate / Power mode */
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);
  lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);

  /* Read samples in polling mode (no int) */


}

/* Main Example --------------------------------------------------------------*/
void nine_dof_read(void)
{
  /* Read device status register */
    lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

    if ( reg.status_imu.xlda && reg.status_imu.gda ) {
      /* Read imu data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
      lsm9ds1_acceleration_raw_get(&dev_ctx_imu,
                                   data_raw_acceleration);
      lsm9ds1_angular_rate_raw_get(&dev_ctx_imu,
                                   data_raw_angular_rate);
      Acceleration_mg[0] = lsm9ds1_from_fs4g_to_mg(
                             data_raw_acceleration[0]);
      Acceleration_mg[1] = lsm9ds1_from_fs4g_to_mg(
                             data_raw_acceleration[1]);
      Acceleration_mg[2] = lsm9ds1_from_fs4g_to_mg(
                             data_raw_acceleration[2]);
      Angular_rate_mdps[0] = lsm9ds1_from_fs2000dps_to_mdps(
                               data_raw_angular_rate[0]);
      Angular_rate_mdps[1] = lsm9ds1_from_fs2000dps_to_mdps(
                               data_raw_angular_rate[1]);
      Angular_rate_mdps[2] = lsm9ds1_from_fs2000dps_to_mdps(
                               data_raw_angular_rate[2]);
    }

    if ( reg.status_mag.zyxda ) {
      /* Read magnetometer data */
      memset(data_raw_magnetic_field, 0x00, 3 * sizeof(int16_t));
      lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field);
      Magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(
                                   data_raw_magnetic_field[0]);
      Magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(
                                   data_raw_magnetic_field[1]);
      Magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(
                                   data_raw_magnetic_field[2]);
    }
}

/*チップセレクトの関数を追加*/
static inline void cs_select(uint16_t cs_pin) {
    //printf("cspin=%d\n",cs_pin);
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect(uint16_t cs_pin) {
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin, 1);
    asm volatile("nop \n nop \n nop");
}
/*
 * @brief  Write generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */

static int32_t platform_write_imu(void *handle, uint8_t reg, 
                                  const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  //HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  //HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  //HAL_SPI_Transmit(sensbus->hbus, (uint8_t*) bufp, len, 1000);
  //HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);

  cs_select(sensbus->cs_pin);
  spi_write_blocking(/*sensbus->hbus*/spi0, &reg, 1);
  spi_write_blocking(/*sensbus->hbus*/spi0, (uint8_t*) bufp, len);
  cs_deselect(sensbus->cs_pin);
  return 0;

}


/*
 * @brief  Write generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write_mag(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;
  /* Write multiple command */

  reg |= 0x40;
  //HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  //HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  //HAL_SPI_Transmit(sensbus->hbus, (uint8_t*) bufp, len, 1000);
  //HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);
  //return 0;

  cs_select(sensbus->cs_pin);
  spi_write_blocking(/*sensbus->hbus*/spi0, &reg, 1);
  spi_write_blocking(/*sensbus->hbus*/spi0, (uint8_t*) bufp, len);
  cs_deselect(sensbus->cs_pin);
  return 0;
}

/*
 * @brief  Read generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_imu(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  /* Read command */
  //reg |= 0x80;
  //HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  //HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  //HAL_SPI_Receive(sensbus->hbus, bufp, len, 1000);
  //HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);
  //return 0;

  reg |= 0x80;
  cs_select(sensbus->cs_pin);
  spi_write_blocking(/*sensbus->hbus*/spi0, &reg, 1);
  spi_read_blocking(/*sensbus->hbus*/spi0, 0, bufp, len);
  cs_deselect(sensbus->cs_pin);
  return 0;

}

/*
 * @brief  Read generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_mag(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  /* Read multiple command */
  //reg |= 0xC0;
  //HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  //HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  //HAL_SPI_Receive(sensbus->hbus, bufp, len, 1000);
  //HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);
  //return 0;

  reg |= 0xC0;
  cs_select(sensbus->cs_pin);
  spi_write_blocking(/*sensbus->hbus*/spi0, &reg, 1);
  spi_read_blocking(/*sensbus->hbus*/spi0, 0, bufp, len);
  cs_deselect(sensbus->cs_pin);
  return 0;

}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  //CDC_Transmit_FS(tx_buffer, len);
  printf("%s",tx_buffer);
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  sleep_ms(ms);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
  /*  
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
  */

  stdio_init_all();
  sleep_ms(1000);

  printf("Hello, LSM9DS1! Reading raw data from registers via SPI...\n");

  // This example will use SPI0 at 0.5MHz.
  spi_init(SENSOR_BUS, 10 * 1000);
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

  // Chip select is active-low, so we'll initialise it to a driven-high state
  gpio_init(PIN_CSAG);
  gpio_init(PIN_CSM);
  gpio_set_dir(PIN_CSAG, GPIO_OUT);
  gpio_set_dir(PIN_CSM, GPIO_OUT);
  gpio_put(PIN_CSAG, 1);
  gpio_put(PIN_CSM, 1);
  sleep_ms(1000);

}



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
  platform_init();
  nine_dof_settei();

 while(1){
} 
 return 0;
    float duty_rr,duty_rl,duty_fl,duty_fr;
    float seigyo=0.5;  
    stdio_init_all(); //シリアル通信の設定
    pwm_settei();
    serial_settei();
    elevator();
    sleep_ms(2000);
    
    while(true){	
        nine_dof_read();
//  printf("%11f %11f %11f %11f %11f %11f %11f %11f %11f\n", Acceleration_mg[0],  Acceleration_mg[1], Acceleration_mg[2], Angular_rate_mdps[0],  Angular_rate_mdps[1], Angular_rate_mdps[2],Magnetic_field_mgauss[0],Magnetic_field_mgauss[1],Magnetic_field_mgauss[2]);
        sleep_ms(10);
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


