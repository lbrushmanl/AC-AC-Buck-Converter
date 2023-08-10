#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "Arduino.h"
#include <QuickPID.h>
#include "EmonLib.h" 

#include <Wire.h>

//DeadtimePWM
//https://forums.raspberrypi.com/viewtopic.php?t=314395
#define PWM_A 5
#define PWM_B 4
#define PWM_C 11
#define PWM_D 10

#define PWM_FREQ 40e3
#define SETPOINT 500
#define VOLTAGE 26
#define CURRENT 27
#define ADC_BITS 12
#define OFFSET 2048

#define LED_PIN 16

#define DEAD_TIME 100

float setpoint, realPower, duty_cycle, Vrms, Irms;
float Kp = .02, Ki = 0.1, Kd = 0; // 1kW Load

QuickPID powerPID(&realPower, &duty_cycle, &setpoint);
EnergyMonitor emon1;                   // Create an instance

uint32_t pwm_set_freq_duty(uint slice_num_1, uint slice_num_2, uint32_t freq, int d)
{
  uint32_t clock = 125000000;
  uint32_t divider16 = clock / freq / 4096 + (clock % (freq * 4096) != 0);

  if (divider16 / 16 == 0)
    divider16 = 16;

  uint32_t wrap = clock * 16 / divider16 / freq - 1;

  pwm_set_clkdiv_int_frac(slice_num_1, divider16/16, divider16 & 0xF);
  pwm_set_clkdiv_int_frac(slice_num_2, divider16/16, divider16 & 0xF);

  pwm_set_output_polarity(slice_num_1, true, true);
  pwm_set_phase_correct(slice_num_1, true);

  pwm_set_output_polarity(slice_num_2, false, false);
  pwm_set_phase_correct(slice_num_2, true);

  pwm_set_wrap(slice_num_1, wrap);
  pwm_set_wrap(slice_num_2, wrap);

//Positive Buck
  pwm_set_chan_level(slice_num_1, PWM_CHAN_B,  (wrap + DEAD_TIME) * d / 100);
  pwm_set_chan_level(slice_num_1, PWM_CHAN_A, 0);

  pwm_set_chan_level(slice_num_2, PWM_CHAN_B, wrap * d / 100);
  pwm_set_chan_level(slice_num_2, PWM_CHAN_A, wrap);

// Negitive Buck
  pwm_set_chan_level(slice_num_1, PWM_CHAN_B, 0);
  pwm_set_chan_level(slice_num_1, PWM_CHAN_A,  (wrap + DEAD_TIME) * d / 100);

  pwm_set_chan_level(slice_num_2, PWM_CHAN_B, wrap);
  pwm_set_chan_level(slice_num_2, PWM_CHAN_A, wrap * d / 100);

//Zero Crossing
  pwm_set_chan_level(slice_num_1, PWM_CHAN_B, 0);
  pwm_set_chan_level(slice_num_1, PWM_CHAN_A, 0);

  pwm_set_chan_level(slice_num_2, PWM_CHAN_B, 0);
  pwm_set_chan_level(slice_num_2, PWM_CHAN_A, 0);
  

  pwm_set_mask_enabled(0x00000000);

  pwm_set_counter(PWM_A, 0);
  pwm_set_counter(PWM_B, 0);
  pwm_set_counter(PWM_C, 0);
  pwm_set_counter(PWM_D, 0);


  pwm_set_mask_enabled(0x000000FF);

  return wrap;
}

int main()
{
  // init();

  uint slice_set_1, slice_set_2, wrap;

  Serial.begin(9600);
  // PWM Setup
  gpio_set_function(PWM_A, GPIO_FUNC_PWM);
  gpio_set_function(PWM_B, GPIO_FUNC_PWM);
  gpio_set_function(PWM_C, GPIO_FUNC_PWM);
  gpio_set_function(PWM_D, GPIO_FUNC_PWM);


  slice_set_1 = pwm_gpio_to_slice_num (PWM_A); 
  slice_set_2 = pwm_gpio_to_slice_num (PWM_D); 

  wrap = pwm_set_freq_duty(slice_set_1, slice_set_2, PWM_FREQ, 20);

  setpoint = 620;
  powerPID.SetOutputLimits(0, 90);
  powerPID.SetTunings(Kp, Ki, Kd);
  powerPID.SetMode(powerPID.Control::automatic);

  // analogReadResolution(ADC_BITS);
  emon1.current(CURRENT, 11);    
  emon1.voltage(VOLTAGE, 240, 1.7);  
  // emon1.current(CURRENT, 42.5);    
  // emon1.voltage(VOLTAGE, 975, 1.7);  

  //Mains power measurment setup
  adc_init();
  adc_gpio_init(VOLTAGE);
  adc_gpio_init(CURRENT);
  adc_select_input(1);

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  uint8_t tik = 0;

  uint8_t duty = 50;
  uint16_t voltage = 4096;
  while(1)
  {
    uint16_t voltage = adc_read();



    if (voltage >= 2200) //2358 @ 20V
    {
      //Positive Buck
      pwm_set_chan_level(slice_set_1, PWM_CHAN_B,  (wrap + DEAD_TIME) * duty / 100);
      pwm_set_chan_level(slice_set_1, PWM_CHAN_A, 0);

      pwm_set_chan_level(slice_set_2, PWM_CHAN_B, wrap * duty / 100);
      pwm_set_chan_level(slice_set_2, PWM_CHAN_A, wrap);
    } else if (voltage <= 1896) // 1738 @ 20V
    {
      // Negitive Buck
      pwm_set_chan_level(slice_set_1, PWM_CHAN_B, 0);
      pwm_set_chan_level(slice_set_1, PWM_CHAN_A,  (wrap + DEAD_TIME) * duty / 100);

      pwm_set_chan_level(slice_set_2, PWM_CHAN_B, wrap);
      pwm_set_chan_level(slice_set_2, PWM_CHAN_A, wrap * duty / 100);
    } else 
    {
      //Zero Crossing
      gpio_put(PWM_A, 1);
      pwm_set_chan_level(slice_set_1, PWM_CHAN_B, wrap);
      pwm_set_chan_level(slice_set_1, PWM_CHAN_A, wrap);

      pwm_set_chan_level(slice_set_2, PWM_CHAN_B, wrap);
      pwm_set_chan_level(slice_set_2, PWM_CHAN_A, wrap);
    }
  }
}