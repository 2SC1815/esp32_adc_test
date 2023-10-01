

#include <rom/lldesc.h>
#include <driver/rtc_io.h>
#include <driver/i2s.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <driver/ledc.h>

const int8_t pin_D7 = 13;
const int8_t pin_D6 = 12;
const int8_t pin_D5 = 14;
const int8_t pin_D4 = 27;
const int8_t pin_D3 = 33;
const int8_t pin_D2 = 32;
const int8_t pin_D1 = 35;
const int8_t pin_D0 = 34;
const int8_t pin_clk = 26;

uint8_t data_buffer[4096];

intr_handle_t i2s_intr_handle;
static void IRAM_ATTR i2s_isr(void* arg);

void init_dac() {
  

  /*--configure signal pins--*/
  
  gpio_matrix_in(pin_D0, I2S0I_DATA_IN0_IDX, false);
  gpio_matrix_in(pin_D1, I2S0I_DATA_IN1_IDX, false);
  gpio_matrix_in(pin_D2, I2S0I_DATA_IN2_IDX, false);
  gpio_matrix_in(pin_D3, I2S0I_DATA_IN3_IDX, false);
  gpio_matrix_in(pin_D4, I2S0I_DATA_IN4_IDX, false);
  gpio_matrix_in(pin_D5, I2S0I_DATA_IN5_IDX, false);
  gpio_matrix_in(pin_D6, I2S0I_DATA_IN6_IDX, false);
  gpio_matrix_in(pin_D7, I2S0I_DATA_IN7_IDX, false);
  
  gpio_matrix_out(pin_clk, I2S0I_BCK_OUT_IDX, false, false);
  

  /*enable i2s mode to parallel  master rx mode */

  
  // Enable and configure I2S peripheral
  periph_module_enable(PERIPH_I2S0_MODULE);

  //reset i2s conf
  const uint32_t lc_conf_reset_flags = I2S_IN_RST_M | I2S_AHBM_RST_M
                                       | I2S_AHBM_FIFO_RST_M;
  I2S0.lc_conf.val |= lc_conf_reset_flags;
  I2S0.lc_conf.val &= ~lc_conf_reset_flags;
 
  const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M
                                    | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
  I2S0.conf.val |= conf_reset_flags;
  I2S0.conf.val &= ~conf_reset_flags;
  while (I2S0.state.rx_fifo_reset_back) {;}
  
  /*--configure clock--*/
  
  //RTCCNTL.ana_conf.plla_force_pu = 1; 
  //rtc_clk_apll_enable(1, 0, 0, 4, 0);                               // enable APLL clock 80 Mhz
  
  //.clkm_sel = 2;         // clock source set to 160 MHz
  
  // i2s clock freq. set to 10 MHz = 160MHz / ( 16 + ( 0 / 1 ) )
  I2S0.clkm_conf.clkm_div_num = 16;   // I2S clock divider’s integral value
  I2S0.clkm_conf.clkm_div_b = 0;      // Fractional clock divider’s numerator value
  I2S0.clkm_conf.clkm_div_a = 1;      // Fractional clock divider’s denominator value
  
  I2S0.clkm_conf.clk_en = 1;          // I2S clock enable
  
 // i2s_set_clk(I2S_NUM_0, 4000000, 16, i2s_channel_tch)
  
  //I2S0.clkm_conf.clka_en = 1;         // Set this bit to enable clk_apll


  // I2S_RX_SLAVE_MOD in register I2S_CONF_REG to 0

  // disable slave mode (sampling clock is not external)
  I2S0.conf.rx_slave_mod = 0;
  
  // Enable parallel mode
  I2S0.conf2.lcd_en = 1;
  
  //set rx data mode

  
  //I2S0.conf2.camera_en = 1;
  
  // FIFO configuration
  I2S0.fifo_conf.rx_fifo_mod = 3; //fifo mode = 3
  I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
  I2S0.conf_chan.rx_chan_mod = 1;
  // Clear flags which are used in I2S serial mode
  I2S0.sample_rate_conf.rx_bits_mod = 0;
  I2S0.conf.rx_right_first = 0;
  I2S0.conf.rx_msb_right = 0;
  I2S0.conf.rx_msb_shift = 0;
  I2S0.conf.rx_mono = 0;
  I2S0.conf.rx_short_sync = 0;
  I2S0.timing.val = 0;
  I2S0.timing.rx_dsync_sw = 1;
  
  esp_intr_alloc(ETS_I2S0_INTR_SOURCE,
                 ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
                 &i2s_isr, NULL, &i2s_intr_handle);
  
}

static void IRAM_ATTR i2s_isr(void* arg){
  I2S0.int_clr.val = I2S0.int_raw.val;
  bool need_yield = false;
  lldesc_t *d = dma_desc;
  int i, pix_cnt = 0;
 
  for (i = 2; i < dma_buf_size; i += 4) {
    data_buffer[pix_cnt++] = (uint8_t) * (d->buf + i);
  }
 
    I2S0.conf.rx_start = 0;
    I2S0.in_link.start = 0;
    stp = false;
}

volatile bool stp = true;

void setup() {

  init_dac();
  
  I2S0.conf.rx_start = 0;
  I2S0.in_link.start = 0;
  esp_intr_disable(i2s_intr_handle);

  
  I2S0.rx_eof_num = 4096;
  I2S0.in_link.addr = (uint32_t)dma_desc;
  I2S0.in_link.start = 1;
  I2S0.conf.rx_start = 1;
  I2S0.int_clr.val = I2S0.int_raw.val;
  I2S0.int_ena.val = 0;
  I2S0.int_ena.in_done = 1;
  esp_intr_enable(i2s_intr_handle);

  while(stp){}
  Serial.begin(9600);
  for(int i=0; i<4096; i++)
    Serial.println(data_buffer[i]);

}

void loop() {
  // put your main code here, to run repeatedly:

}
