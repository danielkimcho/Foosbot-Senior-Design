/**
 ******************************************************************************
 * @file    main.c
 * @author  Gideon Fulton, Emiliano, Alejandro Barroso, Patrick Hadiwinoto
 * @date    October 25, 2024
 * @brief   ECE 362 Team 51 Project
 ******************************************************************************
*/

//added "${workspaceFolder}/.pio/packages/framework-cmsis/Device/ST/STM32F0xx/Include" to includePath in c_cpp_properties.json
//added "STM32F091" to defines in c_cpp_properties.json

#include "stm32f0xx.h"
//#include "stm32f091xc.h" //added instead of above include
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "commands.h"
#include <stdbool.h>

void nano_wait(unsigned int);
//void game(void);
void internal_clock();
void check_wiring();
void autotest();
void setn(int32_t, int32_t);
int32_t readpin(int32_t);
void buttons();
void setup_tim7_for_inputs();
void small_delay();

void init_spi1();
void spi_cmd(unsigned int);
void spi_data(unsigned int);
void spi1_init_oled();
void spi1_display1(const char*);
void spi1_display2(const char*);
void spi1_clear_display();
void update_oled_display(const char*, const char*);
void init_tim15();
void init_tim16();
void setup_bb();
void update_display(int d1, int d2, int d3, int d4);

uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };

#define EXIT_SUCCESS 0
#define THRESHOLD 3500 //THRESHOLD = Vmeasured_analog / Vref * (2^12 - 1) = 3 / 3.3 * 4095 = 3723 which is roughly 3500 (2.82V). threshold is a digital value for comparison
#define PC0_CHANNEL ADC_CHSELR_CHSEL10
#define PA0_CHANNEL ADC_CHSELR_CHSEL0

const char font[] = {
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, // 32: space
    0x86, // 33: exclamation
    0x22, // 34: double quote
    0x76, // 35: octothorpe
    0x00, // dollar
    0x00, // percent
    0x00, // ampersand
    0x20, // 39: single quote
    0x39, // 40: open paren
    0x0f, // 41: close paren
    0x49, // 42: asterisk
    0x00, // plus
    0x10, // 44: comma
    0x40, // 45: minus
    0x80, // 46: period
    0x00, // slash
    // digits
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x67,
    // seven unknown
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    // Uppercase
    0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x6f, 0x76, 0x30, 0x1e, 0x00, 0x38, 0x00,
    0x37, 0x3f, 0x73, 0x7b, 0x31, 0x6d, 0x78, 0x3e, 0x00, 0x00, 0x00, 0x6e, 0x00,
    0x39, // 91: open square bracket
    0x00, // backslash
    0x0f, // 93: close square bracket
    0x00, // circumflex
    0x08, // 95: underscore
    0x20, // 96: backquote
    // Lowercase
    0x5f, 0x7c, 0x58, 0x5e, 0x79, 0x71, 0x6f, 0x74, 0x10, 0x0e, 0x00, 0x30, 0x00,
    0x54, 0x5c, 0x73, 0x7b, 0x50, 0x6d, 0x78, 0x1c, 0x00, 0x00, 0x00, 0x6e, 0x00
};

//===========================================================================
// Configure All Ports
//===========================================================================
void enable_ports(void) {
   RCC->AHBENR |= RCC_AHBENR_GPIOCEN; 

   //enable PC0 for analog input and PC1-PC8 for output
   GPIOC->MODER &= ~0x0003ffff;
   GPIOC->MODER |= 0x00015557;


   RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 

   //enable PA0 for analog input and PA1-PA8 for output
   GPIOA->MODER &= ~0x0003ffff;
   GPIOA->MODER |= 0x00015557;
}

void init_adc(void) {
   RCC->APB2ENR |= RCC_APB2ENR_ADCEN; //ADC interface clock enable

   ADC1->CR |= ADC_CR_ADCAL; //start ADC callibration

   while (ADC1->CR & ADC_CR_ADCAL) { //wait until callibration finishes (goes low)
   }

   ADC1->CR |= ADC_CR_ADEN; //enable ADC

   while (!(ADC1->ISR & ADC_ISR_ADRDY)) { //wait until ardy has been set (goes high)
   }

   GPIOC->ODR = (GPIOC->ODR & 0xfe01) | ((uint16_t)(font['0']) << 1);
   GPIOA->ODR = (GPIOA->ODR & 0xfe01) | ((uint16_t)(font['0']) << 1);
}

uint16_t read_adc(uint32_t channel) {
   while (ADC1->CR & ADC_CR_ADSTART) {
   }
   //Select requested channel
   ADC1->CHSELR = channel;

   //Start conversion
   ADC1->CR |= ADC_CR_ADSTART;

   //Wait for conversion complete
   while (!(ADC1->ISR & ADC_ISR_EOC)) {
   }

   //Read and return result
   return ADC1->DR;
}

void victory_lap(volatile uint32_t* odr) {
   uint32_t segments[] = {
      (1 << 1), 
      (1 << 2), 
      (1 << 3), 
      (1 << 4), 
      (1 << 5), 
      (1 << 6)  
   };

   for (int lap = 0; lap < 5; lap++) {             
      for (int i = 0; i < 6; i++) {               
         *odr = segments[i];         
         nano_wait(25000000);                 
      }
   }

   *odr &= 0xfe01; 
}

void disable_interrupts() {
   GPIOC->ODR &= 0xfe01;
   GPIOA->ODR &= 0xfe01;
   NVIC_DisableIRQ(TIM2_IRQn);
   NVIC_DisableIRQ(TIM7_IRQn);
}

volatile uint8_t consecutive_lows = 0; //number of times/ms in a row that the ball was detected
volatile uint8_t consecutive_highs = 0; //number of times/ms in a row that the ball was NOT detected

volatile uint16_t lockout_ticks = 0; //number of ms after score that interrupt waits before checking for score again
volatile bool is_rearmed = true; //has a high/no ball been detected for at least 20 times/ms in a row since last score

volatile uint8_t goals_detected = 0; //how many goals have been scored
volatile bool is_game_done = false; //is the game finished


void TIM2_IRQHandler(void) {
   //Acknowledge the interrupt by clearing the interrupt flag
   TIM2->SR &= ~TIM_SR_UIF;

   if (is_game_done == false) {
      if (lockout_ticks > 0) {
         lockout_ticks = lockout_ticks - 1;
         return;
      }

      uint16_t pc0 = read_adc(PC0_CHANNEL);

      if (pc0 < THRESHOLD) { //low detected
         consecutive_lows = consecutive_lows + 1;
         consecutive_highs = 0;
         if (consecutive_lows > 4 && is_rearmed) { //low/ball detected for 20 ms straight
            goals_detected = goals_detected + 1;
            consecutive_lows = 0;
            consecutive_highs = 0;
            lockout_ticks = 100; //wait lockout_ticks # ms before checking for goals again
            is_rearmed = false;
            GPIOC->ODR = (GPIOC->ODR & 0xfe01) | ((uint16_t)(font[goals_detected + '0']) << 1); //output to PC1-PC8 for display
         }
      }
      else { //high detected
         consecutive_lows = 0;
         consecutive_highs = consecutive_highs + 1;
         if (consecutive_highs > 19) { //high/no ball detected for 20ms straight
            is_rearmed = true;
         }
      }

      if (goals_detected > 9) { //check if game is finished
         disable_interrupts();
         victory_lap(&(GPIOC->ODR));
         goals_detected = 0;
         is_game_done = true;
      }
   }
}

void init_tim2(void) {

   RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

   TIM2->PSC = 47;  
   TIM2->ARR = 999; 

   TIM2->DIER |= TIM_DIER_UIE;

   TIM2->CR1 |= TIM_CR1_CEN;

   NVIC_EnableIRQ(TIM2_IRQn);
}

volatile uint8_t consecutive_lows_2 = 0; //number of times/ms in a row that the ball was detected
volatile uint8_t consecutive_highs_2 = 0; //number of times/ms in a row that the ball was NOT detected

volatile uint16_t lockout_ticks_2 = 0; //number of ms after score that interrupt waits before checking for score again
volatile bool is_rearmed_2 = true; //has a high/no ball been detected for at least 20 times/ms in a row since last score

volatile uint8_t goals_detected_2 = 0; //how many goals have been scored
volatile bool is_game_done_2 = false; //is the game finished


void TIM7_IRQHandler(void) {
   //Acknowledge the interrupt by clearing the interrupt flag
   TIM7->SR &= ~TIM_SR_UIF;

   if (is_game_done_2 == false) {
      if (lockout_ticks_2 > 0) {
         lockout_ticks_2 = lockout_ticks_2 - 1;
         return;
      }

      uint16_t pa0 = read_adc(PA0_CHANNEL);

      if (pa0 < THRESHOLD) { //low detected
         consecutive_lows_2 = consecutive_lows_2 + 1;
         consecutive_highs_2 = 0;
         if (consecutive_lows_2 > 4 && is_rearmed_2) { //low/ball detected for 20 ms straight
            goals_detected_2 = goals_detected_2 + 1;
            consecutive_lows_2 = 0;
            consecutive_highs_2 = 0;
            lockout_ticks_2 = 100; //wait lockout_ticks # ms before checking for goals again
            is_rearmed_2 = false;
            GPIOA->ODR = (GPIOA->ODR & 0xfe01) | ((uint16_t)(font[goals_detected_2 + '0']) << 1); //output to PC1-PC8 for display
         }
      }
      else { //high detected
         consecutive_lows_2 = 0;
         consecutive_highs_2 = consecutive_highs_2 + 1;
         if (consecutive_highs_2 > 19) { //high/no ball detected for 20ms straight
            is_rearmed_2 = true;
         }
      }

      if (goals_detected_2 > 9) { //check if game is finished
         disable_interrupts();
         victory_lap(&(GPIOA->ODR));
         goals_detected_2 = 0;
         is_game_done_2 = true;
      }
   }
}

void init_tim7(void) {

   RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

   TIM7->PSC = 47;  
   TIM7->ARR = 999; 

   TIM7->DIER |= TIM_DIER_UIE;

   TIM7->CR1 |= TIM_CR1_CEN;

   NVIC_EnableIRQ(TIM7_IRQn);
}



void enable_rotational_motor_ports(void) {
   RCC->AHBENR |= RCC_AHBENR_GPIOCEN; 

   GPIOC->MODER &= ~0x03fc0000;
   GPIOC->MODER |= 0x000140000;
}

volatile uint16_t pulse_target = 0;
volatile uint16_t num_pulses = 0;
volatile bool is_waiting_for_next_pulse = true;
volatile bool clockwise = false;

void TIM3_IRQHandler(void) {

   if ((GPIOC->IDR & 0x0800) && is_waiting_for_next_pulse) { //pulse is high, count new pulse
      num_pulses += 1;
      is_waiting_for_next_pulse = false;
   }
   else if (!(GPIOC->IDR & 0x0800)) { //pulse is low, so waiting for next pulse
      is_waiting_for_next_pulse = true;
   }

   if (!(GPIOC->ODR & 0x0200) && num_pulses < pulse_target && clockwise) { //if cw rotation and rotation has not been started, start cw
      GPIOC->ODR = 0x0200;
   }
   else if (!(GPIOC->ODR & 0x0400) && num_pulses < pulse_target && (!clockwise)) { //if ccw rotation and rotation has not been started, start ccw
      GPIOC->ODR = 0x0600;
   }
   else if (!(GPIOC->ODR == 0x0400) && num_pulses >= pulse_target && clockwise) { //stop cw rotation
      GPIOC->ODR = 0x0600;
      nano_wait(5000000);
      GPIOC->ODR = 0x0400;
   }
   else if (!(GPIOC->ODR == 0x0000) && num_pulses >= pulse_target && (!clockwise)) { //stop ccw rotation
      GPIOC->ODR = 0x0200;
      nano_wait(5000000);
      GPIOC->ODR = 0x0000;
   }
}

void init_tim3(void) {
   RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

   TIM3->PSC = 47;  
   TIM3->ARR = 499; 

   TIM3->DIER |= TIM_DIER_UIE;

   TIM3->CR1 |= TIM_CR1_CEN;

   NVIC_EnableIRQ(TIM3_IRQn);
}

void spin(int degrees) {
   int sign = (degrees > 0) - (degrees < 0);

   if (0.9348 * sign * degrees - 10.6 < 0) {
      pulse_target = 0; 
   }
   else {
      pulse_target = (int) (0.9348 * sign * degrees - 10.6);
   }

   if (sign < 0) {
      clockwise = true;
   }
   else {
      clockwise = false;
   }
}





void enable_linear_motor_ports() {
   RCC->AHBENR |= RCC_AHBENR_GPIOCEN; 

   //CHANGE
   GPIOC->MODER &= ~0x03fc0000;
   GPIOC->MODER |= 0x000140000;
}

void init_tim14(void) {
   RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

   TIM14->PSC = 47;  
   TIM14->ARR = 999; 

   TIM14->DIER |= TIM_DIER_UIE;

   TIM14->CR1 |= TIM_CR1_CEN;

   NVIC_EnableIRQ(TIM14_IRQn);
}

void TIM14_IRQHandler(void) {
   
}

void extend(int mm) {   

}



int main(void) {

   /*
   enable_ports();
   init_adc();
   init_tim2();
   init_tim7();
   */


   /*
   int degrees = 2000;

   enable_rotational_motor_ports();
   spin(degrees);
   init_tim3();
   */

   int mm = 1000;

   enable_linear_motor_ports();
   extend(mm);
   init_tim14();


   

   return EXIT_SUCCESS;
}
