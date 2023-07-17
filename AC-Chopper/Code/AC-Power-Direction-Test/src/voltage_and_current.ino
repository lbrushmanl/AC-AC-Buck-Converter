#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "Arduino.h"
#include <QuickPID.h>
#include "EmonLib.h" 


// #include <SPI.h>
// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>


// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels

// #define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
// #define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3C 

#define PWM 19
#define PWM_FREQ 20e3
#define SETPOINT 500
#define VOLTAGE 26
#define CURRENT 27
#define ADC_BITS 12
#define OFFSET 2048

// #define PIN_WIRE0_SDA  (4u)
// #define PIN_WIRE0_SCL  (5u)

float setpoint, realPower, duty_cycle, Vrms, Irms;
float Kp = .02, Ki = 0.1, Kd = 0; // 1kW Load
// float Kp = .09, Ki = 1, Kd = .01; LIGHT BULB
uint slice, channel, wrap;

// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

QuickPID powerPID(&realPower, &duty_cycle, &setpoint);
EnergyMonitor emon1;                   // Create an instance

uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t freq, int d)
{
 uint32_t clock = 125000000;
 uint32_t divider16 = clock / freq / 4096 + 
                           (clock % (freq * 4096) != 0);
 if (divider16 / 16 == 0)
 divider16 = 16;
 uint32_t wrap = clock * 16 / divider16 / freq - 1;
 pwm_set_clkdiv_int_frac(slice_num, divider16/16,
                                     divider16 & 0xF);
 pwm_set_wrap(slice_num, wrap);
 pwm_set_chan_level(slice_num, chan, wrap * d / 100);
 return wrap;
}

// void update_display(float power, float Vrms, float Irms)
// {
//   display.clearDisplay();
//   display.setCursor(0,10);   
//   display.print(power);
//   display.println("W");
//   display.print(Vrms);
//   display.println("V");
//   display.print(Irms);
//   display.println("A");
//   display.display();
// }

// double rmsI() 
// {
//   uint32_t sumI = 0;
//   int16_t current = 0;
//   for (uint16_t i = 0; i < 50000; i++)
//   {
//     current = (adc_read() - OFFSET);
//     if (current < 60)
//       current = 0;

//     sumI += current * current;
//   }

//   return sqrt(sumI / 50000.0f);
  
// }

void setup ()
{
  Serial.begin(9600);
  // PWM Setup
  gpio_set_function(PWM, GPIO_FUNC_PWM);
  slice = pwm_gpio_to_slice_num (PWM); 
  channel = pwm_gpio_to_channel (PWM);
  wrap = pwm_set_freq_duty(slice, channel, PWM_FREQ, 50);
  pwm_set_enabled(slice, true);

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
}

uint8_t tik = 0;

void loop() 
{
  
  if (tik == 10) {
    if (Serial.available() > 0) {
      setpoint = Serial.parseInt();
    }
    tik = 0;
  }

  emon1.calcVI(5,150);
  realPower=emon1.realPower;
  Serial.println(realPower);
  powerPID.Compute();
  // duty_cycle = 20; 
  pwm_set_chan_level(slice, channel, wrap * (100-duty_cycle) / 100);

  tik++;
}


// void pwm()
// {
//   // Get the devider for PWM_1 value derived from CPU
//   uint32_t f_sys = clock_get_hz(clk_sys); // typically 125'000'000 Hz
//   float divider = f_sys / PWM_FREQ;

//   // Set for PWM_1
//   pwm_set_clkdiv(pwmConfig.sliceOne, divider); // Here divider = 1
//   pwmConfig.top1 =  (uint16_t)(PWM_FREQ/PWM_FREQ -1);  // PWM_FREQ = 400000 to get 200KHz in Phase Correct

//   // A --> Non inverting, B --> Inverting
//   pwm_set_output_polarity(pwmConfig.sliceOne, false, true);
//   // Phase correct mode
//   pwm_set_phase_correct(pwmConfig.sliceOne, true);

//   // Set PWM Wrap
//   pwm_set_wrap(pwmConfig.sliceOne, pwmConfig.top1);

//   // Testing the PWM (NO IRQ)
//   pwmConfig.dutyCh1 = (uint16_t)100;
//   pwmConfig.dutyCh2 = (uint16_t)140;   // 40 added as offset for dead band

//   // Test**************************************

//   pwm_set_chan_level(pwmConfig.sliceOne, PWM_CHAN_A, (uint16_t)pwmConfig.dutyCh1);
//   pwm_set_chan_level(pwmConfig.sliceOne, PWM_CHAN_B, (uint16_t)pwmConfig.dutyCh2);
//   printf("CPU Clock: %d, Devider: %.2f, Top1: %d, Top2: %d, Duty1: %d, Duty2: %d\n",
//           f_sys, divider, pwmConfig.top1, pwmConfig.top2,pwmConfig.dutyCh1, pwmConfig.dutyCh2);
//   // Enable the PWM
//   pwm_set_enabled(pwmConfig.sliceOne, true); // let's go!
// }

