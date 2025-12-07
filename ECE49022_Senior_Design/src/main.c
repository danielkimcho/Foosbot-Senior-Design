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
#include <assert.h>
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


void tic_set_target_position(USART_TypeDef *USARTx, int32_t target);
void tic_exit_safe_start(USART_TypeDef *USARTx);
void disable_interrupts(void);

uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };

#define EXIT_SUCCESS 0
#define THRESHOLD_TIM2 620 //3500 //THRESHOLD = Vmeasured_analog / Vref * (2^12 - 1) = 3 / 3.3 * 4095 = 3723 which is roughly 3500 (2.82V). threshold is a digital value for comparison
#define THRESHOLD_TIM7 620 //3500 //THRESHOLD = Vmeasured_analog / Vref * (2^12 - 1) = 3 / 3.3 * 4095 = 3723 which is roughly 3500 (2.82V). threshold is a digital value for comparison
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

   //GPIOC->ODR = (GPIOC->ODR & 0xfe01) | ((uint16_t)(font['0']) << 1);
   GPIOC->BSRR = (0x1FE << 16);                         // clear PC1–PC8
   GPIOC->BSRR = ((uint16_t)(font['0']) << 1) & 0x1FE;  // set new segment bits

   //GPIOA->ODR = (GPIOA->ODR & 0xfe01) | ((uint16_t)(font['0']) << 1);
   GPIOA->BSRR = (0x1FE << 16);                         // clear PA1–PA8
   GPIOA->BSRR = ((uint16_t)(font['0']) << 1) & 0x1FE;  // set new segment bits
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

void victory_lap(volatile uint32_t* odr) { //FIX
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

      if (pc0 < THRESHOLD_TIM2) { //low detected
         consecutive_lows = consecutive_lows + 1;
         consecutive_highs = 0;
         if (consecutive_lows > 4 && is_rearmed) { //low/ball detected for 20 ms straight
            goals_detected = goals_detected + 1;
            consecutive_lows = 0;
            consecutive_highs = 0;
            lockout_ticks = 100; //wait lockout_ticks # ms before checking for goals again
            is_rearmed = false;
            //GPIOC->ODR = (GPIOC->ODR & 0xfe01) | ((uint16_t)(font[goals_detected + '0']) << 1); //output to PC1-PC8 for display //FIX
            GPIOC->BSRR = (0x1FE << 16);                         // clear PA1–PA8
            GPIOC->BSRR = ((uint16_t)(font[goals_detected + '0']) << 1) & 0x1FE;  // set new segment bits
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
         //center_rods();
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

      if (pa0 < THRESHOLD_TIM7) { //low detected
         consecutive_lows_2 = consecutive_lows_2 + 1;
         consecutive_highs_2 = 0;
         if (consecutive_lows_2 > 4 && is_rearmed_2) { //low/ball detected for 20 ms straight
            goals_detected_2 = goals_detected_2 + 1;
            consecutive_lows_2 = 0;
            consecutive_highs_2 = 0;
            lockout_ticks_2 = 100; //wait lockout_ticks # ms before checking for goals again
            is_rearmed_2 = false;
            //GPIOA->ODR = (GPIOA->ODR & 0xfe01) | ((uint16_t)(font[goals_detected_2 + '0']) << 1); //output to PC1-PC8 for display //FIX
            GPIOA->BSRR = (0x1FE << 16);                         // clear PA1–PA8
            GPIOA->BSRR = ((uint16_t)(font[goals_detected_2 + '0']) << 1) & 0x1FE;  // set new segment bits
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
         //center_rods();
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

   GPIOC->MODER &= ~0x00fc0000; //PC9, 10 output PC11 input
   GPIOC->MODER |= 0x00140000;


   RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 

   GPIOB->MODER &= ~0x0000003f; //PB0, PB1 output PB2 input
   GPIOB->MODER |= 0x000000005;
}

volatile uint16_t pulse_target1 = 0;
volatile uint16_t num_pulses1 = 0;
volatile bool is_waiting_for_next_pulse1 = true;
volatile bool clockwise1 = false;
volatile bool is_rotating1 = false;
volatile bool is_stopping1 = false;
volatile uint16_t countdown1 = -1;

void TIM3_IRQHandler(void) {
   TIM3->SR &= ~TIM_SR_UIF;

   if ((GPIOC->IDR & 0x0800) && is_waiting_for_next_pulse1) { //pulse is high, count new pulse
      num_pulses1 += 1;
      is_waiting_for_next_pulse1 = false;
   }
   else if (!(GPIOC->IDR & 0x0800)) { //pulse is low, so waiting for next pulse
      is_waiting_for_next_pulse1 = true;
   }

   if (is_stopping1 && (countdown1 > 0)) {
      countdown1 = countdown1 - 1;
   }
   else if (countdown1 == 0) {
      is_stopping1 = false;
      countdown1 = 5; //(uint16_t)((5000000ULL * 48000000ULL) / ((TIM3->PSC + 1) * (TIM3->ARR + 1) * 1000000000ULL));
      //countdown1 = (5000000UL / (1000000000UL) * (48000000UL / (TIM3->PSC + 1) / (TIM3->ARR + 1)));
      //GPIOC->ODR = 0x0000; //FIX
      GPIOC->BSRR = ((1 << 9) | (1 << 10)) << 16;   // reset PC9, PC10
   }
   else if (!(is_rotating1) && (num_pulses1 < pulse_target1) && clockwise1) { //if cw rotation and rotation has not been started, start cw GPIOC->ODR & 0x0200
      //GPIOC->ODR = 0x0200; //FIX
      GPIOC->BSRR = ((1 << 10) << 16) | (1 << 9);   // reset PC10, set PC9
      is_rotating1 = true;
   }
   else if (!(is_rotating1) && (num_pulses1 < pulse_target1) && (!clockwise1)) { //if ccw rotation and rotation has not been started, start ccw GPIOC->ODR & 0x0400
      //GPIOC->ODR = 0x0600; //FIX
      GPIOC->BSRR = (1 << 9) | (1 << 10);           // set PC9, set PC10
      is_rotating1 = true;
   }
   else if (is_rotating1 && (num_pulses1 >= pulse_target1) && clockwise1) { //stop cw rotation !(GPIOC->ODR == 0x0400)
      //GPIOC->ODR = 0x0600; //FIX
      GPIOC->BSRR = (1 << 9) | (1 << 10);           // set PC9, set PC10
      is_stopping1 = true;
      countdown1 = 5; //(uint16_t)((5000000ULL * 48000000ULL) / ((TIM3->PSC + 1) * (TIM3->ARR + 1) * 1000000000ULL));
      //countdown1 = (5000000UL / (1000000000UL) * (48000000UL / (TIM3->PSC + 1) / (TIM3->ARR + 1)));
      //nano_wait(5000000);
      //nano_wait(500000);
      //GPIOC->ODR = 0x0400;
      is_rotating1 = false; //TODO: may need to change this since motor does not instantly stop
   }
   else if (is_rotating1 && (num_pulses1 >= pulse_target1) && (!clockwise1)) { //stop ccw rotation !(GPIOC->ODR == 0x0000)
      //GPIOC->ODR = 0x0200; //FIX
      GPIOC->BSRR = ((1 << 10) << 16) | (1 << 9);   // reset PC10, set PC9
      is_stopping1 = true;
      countdown1 = 5; //(uint16_t)((5000000ULL * 48000000ULL) / ((TIM3->PSC + 1) * (TIM3->ARR + 1) * 1000000000ULL));
      //countdown1 = (5000000UL / (1000000000UL) * (48000000UL / (TIM3->PSC + 1) / (TIM3->ARR + 1)));
      //nano_wait(5000000);
      //nano_wait(500000);
      //GPIOC->ODR = 0x0000;
      is_rotating1 = false; //TODO: may need to change this since motor does not instantly stop
   }
}

void init_tim3(void) {
   RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

   TIM3->PSC = 47;  
   TIM3->ARR = 499; //999

   TIM3->DIER |= TIM_DIER_UIE;

   TIM3->CR1 |= TIM_CR1_CEN;

   NVIC_EnableIRQ(TIM3_IRQn);
   NVIC_SetPriority(TIM3_IRQn, 2);
}

void spin_motor1(int degrees) {
   int sign = (degrees > 0) - (degrees < 0);
   num_pulses1 = 0;

   if (0.9348 * sign * degrees - 10.6 < 0) {
      pulse_target1 = 0; 
   }
   else {
      pulse_target1 = (int) (0.9348 * sign * degrees - 10.6);
   }

   if (sign < 0) {
      clockwise1 = true;
   }
   else {
      clockwise1 = false;
   }
}



volatile uint16_t pulse_target2 = 0;
volatile uint16_t num_pulses2 = 0;
volatile bool is_waiting_for_next_pulse2 = true;
volatile bool clockwise2 = false;
volatile bool is_rotating2 = false;
volatile bool is_stopping2 = false;
volatile uint16_t countdown2 = -1;

void TIM14_IRQHandler(void) {
   TIM14->SR &= ~TIM_SR_UIF;

   if ((GPIOB->IDR & 0x0004) && is_waiting_for_next_pulse2) { //pulse is high, count new pulse
      num_pulses2 += 1;
      is_waiting_for_next_pulse2 = false;
   }
   else if (!(GPIOB->IDR & 0x0004)) { //pulse is low, so waiting for next pulse
      is_waiting_for_next_pulse2 = true;
   }

   if (is_stopping2 && (countdown2 > 0)) {
      countdown2 = countdown2 - 1;
   }
   else if (countdown2 == 0) {
      is_stopping2 = false;
      countdown2 = 5; //(uint16_t)((5000000ULL * 48000000ULL) / ((TIM3->PSC + 1) * (TIM3->ARR + 1) * 1000000000ULL));
      //countdown2 = (5000000UL / (1000000000UL) * (48000000UL / (TIM3->PSC + 1) / (TIM3->ARR + 1)));
      //GPIOB->ODR = 0x0000; //FIX
      GPIOB->BSRR = ((1 << 0) | (1 << 1)) << 16;    // reset PB0, PB1
   }
   else if (!(is_rotating2) && num_pulses2 < pulse_target2 && clockwise2) { //if cw rotation and rotation has not been started, start cw GPIOB->ODR & 0x0001
      //GPIOB->ODR = 0x0001; //FIX
      GPIOB->BSRR = ((1 << 1) << 16) | (1 << 0);    // reset PB1, set PB0
      is_rotating2 = true;
   }
   else if (!(is_rotating2) && num_pulses2 < pulse_target2 && (!clockwise2)) { //if ccw rotation and rotation has not been started, start ccw GPIOB->ODR & 0x0002
      //GPIOB->ODR = 0x0003; //FIX
      GPIOB->BSRR = (1 << 0) | (1 << 1);            // set PB0, set PB1
      is_rotating2 = true;
   }
   else if (is_rotating2 && num_pulses2 >= pulse_target2 && clockwise2) { //stop cw rotation !(GPIOB->ODR == 0x0002)
      //GPIOB->ODR = 0x0003; //FIX
      GPIOB->BSRR = (1 << 0) | (1 << 1);            // set PB0, set PB1
      is_stopping2 = true;
      countdown2 = 5;
      //nano_wait(5000000);
      //nano_wait(500000);
      //GPIOB->ODR = 0x0002;
      is_rotating2 = false; //TODO: may need to change this since motor does not instantly stop
   }
   else if (is_rotating2 && num_pulses2 >= pulse_target2 && (!clockwise2)) { //stop ccw rotation !(GPIOB->ODR == 0x0000)
      //GPIOB->ODR = 0x0001; //FIX
      GPIOB->BSRR = ((1 << 1) << 16) | (1 << 0);    // reset PB1, set PB0
      is_stopping2 = true;
      countdown2 = 5;
      //nano_wait(5000000);
      //nano_wait(500000);
      //GPIOB->ODR = 0x0000;
      is_rotating2 = false; //TODO: may need to change this since motor does not instantly stop
   }
}

void init_tim14(void) {
   RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

   TIM14->PSC = 47;  
   TIM14->ARR = 499; //999

   TIM14->DIER |= TIM_DIER_UIE;

   TIM14->CR1 |= TIM_CR1_CEN;

   NVIC_EnableIRQ(TIM14_IRQn);
   NVIC_SetPriority(TIM14_IRQn, 2);
}

void spin_motor2(int degrees) {
   int sign = (degrees > 0) - (degrees < 0);
   num_pulses2 = 0;

   if (0.9348 * sign * degrees - 10.6 < 0) {
      pulse_target2 = 0; 
   }
   else {
      pulse_target2 = (int) (0.9348 * sign * degrees - 10.6);
   }

   if (sign < 0) {
      clockwise2 = true;
   }
   else {
      clockwise2 = false;
   }
}




#define MOTOR_LINEAR_1   0
#define MOTOR_LINEAR_2   1
#define MOTOR_ROT_1      2
#define MOTOR_ROT_2      3
#define SYS_CLK_HZ   48000000UL
#define BAUDRATE     19200UL //Baud rate is actually 6x this

void init_usart1(void) {
    // --- Enable clocks ---
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;    // GPIOA clock

    // --- Configure PA9 (TX) and PA10 (RX) as AF1 ---
    GPIOA->MODER &= ~((3 << (9 * 2)) | (3 << (10 * 2))); // clear mode
    GPIOA->MODER |=  ((2 << (9 * 2)) | (2 << (10 * 2))); // AF mode

    GPIOA->AFR[1] &= ~((0xF << ((9 - 8) * 4)) | (0xF << ((10 - 8) * 4)));
    GPIOA->AFR[1] |=  ((0x1 << ((9 - 8) * 4)) | (0x1 << ((10 - 8) * 4))); // AF1

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // USART1 clock

    // --- Configure USART1 ---
    //USART1->CR1 = 0;  // Disable before configuration
    USART1->CR1 &= ~USART_CR1_UE;

    USART1->CR1 &= ~((1 << 28) | (1 << 12)); 

    //Set it for one stop bit.
    USART1->CR2 &= ~(3 << 12);

    //Set it for no parity control.
    USART1->CR1 &= ~(1 << 10); 

   
    //Use 16x oversampling.
    USART1->CR1 &= ~(1 << 15); 


    // Baud rate: BRR = Fck / Baud
    USART1->BRR = SYS_CLK_HZ / BAUDRATE / 36;  // 48000000 / 19200 = 2500 (0x9C4)

    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE; // Enable TX and RX
    USART1->CR1 |= USART_CR1_UE;                // Enable USART

   while ((USART1->ISR & (USART_ISR_TEACK | USART_ISR_REACK)) != (USART_ISR_TEACK | USART_ISR_REACK)) {
   }
}

void init_usart3(void) {
    // --- Enable clocks ---
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;    // Enable GPIOB clock

    // --- Configure PB10 (TX) and PB11 (RX) as AF1 ---
    GPIOB->MODER &= ~((3 << (10 * 2)) | (3 << (11 * 2) )); // clear mode bits
    GPIOB->MODER |=  ((2 << (10 * 2)) | (2 << (11 * 2))); // set AF mode

    GPIOB->AFR[1] &= ~((0xF << ((10 - 8) * 4)) | (0xF << ((11 - 8) * 4))); // clear AF
    GPIOB->AFR[1] |= ((0x4 << ((10 - 8) * 4)) | (0x4 << ((11 - 8) * 4))); // AF4

    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;  // Enable USART3 clock

    // --- Configure USART3 ---
    USART3->CR1 &= ~USART_CR1_UE;   // Disable before config

    USART3->CR1 &= ~((1 << 28) | (1 << 12));  // 8-bit data, no advanced features
    USART3->CR2 &= ~(3 << 12);                // 1 stop bit
    USART3->CR1 &= ~(1 << 10);                // No parity
    USART3->CR1 &= ~(1 << 15);                // 16x oversampling

    // --- Baud rate ---
    USART3->BRR = SYS_CLK_HZ / BAUDRATE / 36;  // same formula as your USART3

    // Enable TX and RX
    USART3->CR1 |= USART_CR1_TE | USART_CR1_RE;

    // Enable USART
    USART3->CR1 |= USART_CR1_UE;

    // Wait for TEACK and REACK
    while ((USART3->ISR & (USART_ISR_TEACK | USART_ISR_REACK)) != (USART_ISR_TEACK | USART_ISR_REACK)) {
    }
}

void init_usart5(void) {
    // --- Enable GPIOB clock ---
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // --- Configure PB3 (TX) and PB4 (RX) as AF4 ---
    GPIOB->MODER &= ~((3 << (3 * 2)) | (3 << (4 * 2)));  // Clear mode bits
    GPIOB->MODER |=  ((2 << (3 * 2)) | (2 << (4 * 2)));  // Alternate function mode

    GPIOB->AFR[0] &= ~((0xF << (3 * 4)) | (0xF << (4 * 4))); // Clear AFR
    GPIOB->AFR[0] |=  ((0x4 << (3 * 4)) | (0x4 << (4 * 4))); // AF4 = USART5

    // --- Enable USART5 clock ---
    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;

    // --- Configure USART5 ---
    USART5->CR1 &= ~USART_CR1_UE;  // Disable USART before config

    USART5->CR1 &= ~((1 << 28) | (1 << 12)); // 8 data bits, no advanced features
    USART5->CR2 &= ~(3 << 12);               // 1 stop bit
    USART5->CR1 &= ~(1 << 10);               // No parity
    USART5->CR1 &= ~(1 << 15); 

    // Baud rate
    USART5->BRR = SYS_CLK_HZ / BAUDRATE / 36;

    // Enable RX and Tx
    USART5->CR1 |= USART_CR1_RE | USART_CR1_TE;

    // Enable RX interrupt so USART3_8_IRQHandler() will fire
    USART5->CR1 |= USART_CR1_RXNEIE;

    // Enable USART
    USART5->CR1 |= USART_CR1_UE;

    // Wait for receiver to be ready
    while (!(USART5->ISR & USART_ISR_REACK)) {
    }

    NVIC_EnableIRQ(USART3_8_IRQn);
}

void usart_send_byte(USART_TypeDef *USARTx, uint8_t byte) {
    while (!(USARTx->ISR & USART_ISR_TXE)) {
        // Wait until transmit data register is empty
    }
    USARTx->TDR = byte;
}

void usart_send_array(USART_TypeDef *USARTx, const uint8_t *data, uint16_t len) {
   for (uint16_t i = 0; i < len; i++) { //linear motor2
      usart_send_byte(USARTx, data[i]);
   }
}

void tic_exit_safe_start(USART_TypeDef *USARTx) {
    uint8_t cmd = 0x83; // Exit Safe Start
    usart_send_byte(USARTx, cmd);
}

void tic_energize(USART_TypeDef *USARTx) {
    uint8_t cmd = 0x85; // Energize
    usart_send_byte(USARTx, cmd);
}

void tic_set_target_position(USART_TypeDef *USARTx, int32_t target) {
    //uint8_t cmd[6];
    uint8_t command = 0xE0; // Set target position
    
    uint8_t data1 = target & 0xFF;
    uint8_t data2 = (target >> 8) & 0xFF;
    uint8_t data3 = (target >> 16) & 0xFF;
    uint8_t data4 = (target >> 24) & 0xFF;
    
    // Build MSBs byte: bits 0–3 contain the MSb of each data byte
    uint8_t msbs = ((target >> 7) & 0x01) | ((target >> 14) & 0x02) | ((target >> 21) & 0x04) | ((target >> 28) & 0x08);   

    data1 &= 0x7F;
    data2 &= 0x7F;
    data3 &= 0x7F;
    data4 &= 0x7F;

    uint8_t data_array[6] = {command, msbs, data1, data2, data3, data4};

    usart_send_array(USARTx, data_array, 6);
}


uint8_t usart_read_byte(USART_TypeDef *USARTx) {
    while (!(USARTx->ISR & USART_ISR_RXNE)) {
    }
    return (uint8_t)(USARTx->RDR & 0xFF);
}

bool is_motor_busy(USART_TypeDef *USARTx) {
    uint8_t cmd[2] = {0xA1, 0x09}; // Get variable: Planning mode
    usart_send_array(USARTx, cmd, 2);

    // Wait for 1-byte response from Tic
    uint8_t response = usart_read_byte(USARTx);

    // Bit 0 indicates whether the Tic is actively planning motion
    return (response != 0); // true = moving, false = target reached
}



#define RX_BUFFER_SIZE 64
volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_write_idx = 0;
volatile uint8_t rx_read_idx = 0;

void USART3_8_IRQHandler(void) { 
   if (USART5->ISR & USART_ISR_RXNE) {
      uint8_t byte = USART5->RDR & 0xFF;
      uint8_t next_rx_write_idx = (rx_write_idx + 1) % RX_BUFFER_SIZE;
      if (next_rx_write_idx != rx_read_idx) { //valid write  
         rx_buffer[rx_write_idx] = byte;
         rx_write_idx = next_rx_write_idx;
      }
   }
}

void usart5_read_byte_nonblocking(uint8_t *byte) {
    *byte = rx_buffer[rx_read_idx];
    rx_read_idx = (rx_read_idx + 1) % RX_BUFFER_SIZE;
}

void handle_data_from_pi(void) {
   /*uint8_t motor_id = usart5_read_byte();
   uint8_t data_byte_1  = usart5_read_byte();
   uint8_t data_byte_2 = usart5_read_byte();

   int16_t target = (int16_t)((data_byte_2 << 8) | data_byte_1);*/

   uint8_t motor_id, data1, data2;

   // Check if at least 3 bytes are available in the buffer
   if ((rx_write_idx - rx_read_idx + RX_BUFFER_SIZE) % RX_BUFFER_SIZE < 3) {
      return; // Not enough data yet, just exit
   }

   // Read one command (3 bytes) non-blocking
   usart5_read_byte_nonblocking(&motor_id);
   usart5_read_byte_nonblocking(&data1);
   usart5_read_byte_nonblocking(&data2);

   int16_t target = (int16_t)((data2 << 8) | data1);
   //printf("%d", motor_id);
   //printf("%d", target);

   switch (motor_id) {
      case MOTOR_LINEAR_1:
         //if (!(is_motor_busy(USART1))) {
         tic_exit_safe_start(USART1);
         //nano_wait(10000);
         tic_set_target_position(USART1, target);
         //nano_wait(500000000);
         //spin_motor1(360);
         //}
         break;
      case MOTOR_LINEAR_2:
         //if (!(is_motor_busy(USART3))) {
         tic_exit_safe_start(USART3);
         //nano_wait(10000);
         tic_set_target_position(USART3, target);
         //spin_motor2(360);
         //}
         break;
      case MOTOR_ROT_1:
         //if (!(is_rotating1)) {
         spin_motor1(target);
         //}
         break;
      case MOTOR_ROT_2:
         //if (!(is_rotating2)) {
         spin_motor2(target);
         //}
         break;
      default:
         break;
   }
}

void TIM6_IRQHandler(void) {
   TIM6->SR &= ~TIM_SR_UIF;
   handle_data_from_pi();
}

void init_tim6(void) {
   RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

   TIM6->PSC = 47;  
   TIM6->ARR = 99999; 

   TIM6->DIER |= TIM_DIER_UIE;

   TIM6->CR1 |= TIM_CR1_CEN;

   NVIC_EnableIRQ(TIM6_IRQn);
   NVIC_SetPriority(TIM6_IRQn, 1);
}

void center_rods(void) {
   
   //RESET POSITION TO MIDDLE
   tic_exit_safe_start(USART1); 
   tic_exit_safe_start(USART3);
   nano_wait(10000);
   tic_set_target_position(USART1, 5000); 
   tic_set_target_position(USART3, 5000); 
   nano_wait(500000000);
   tic_exit_safe_start(USART1); 
   tic_exit_safe_start(USART3);
   nano_wait(10000);
   tic_set_target_position(USART1, -5000); 
   tic_set_target_position(USART3, -5000);
   nano_wait(500000000);
   tic_exit_safe_start(USART1); 
   tic_exit_safe_start(USART3);
   nano_wait(10000);
   tic_set_target_position(USART1, 0); 
   tic_set_target_position(USART3, 0);
}

void disable_interrupts() {
   GPIOC->ODR &= 0xfe01; //FIX
   GPIOA->ODR &= 0xfe01; //FIX
   NVIC_DisableIRQ(TIM2_IRQn);
   NVIC_DisableIRQ(TIM7_IRQn);
   NVIC_DisableIRQ(TIM3_IRQn);
   NVIC_DisableIRQ(TIM14_IRQn);
   NVIC_DisableIRQ(USART3_8_IRQn);
   NVIC_DisableIRQ(TIM6_IRQn);
}




int main(void) {

   //TEST SCORING SYSTEM
   //THIS WILL BE ON FOR FINAL PRODUCT
   enable_ports();
   init_adc();
   init_tim2();
   init_tim7();

   //GPIOC->ODR = (GPIOC->ODR & 0xfe01) | ((uint16_t)(font[9 + '0']) << 1);
   //GPIOC->ODR = (GPIOC->ODR & 0xfe01) | ((uint16_t)(font[goals_detected + '0']) << 1);
   

   
   //Rotation Motors
   //THIS WILL BE ON FOR FINAL PRODUCT
   enable_rotational_motor_ports();
   init_tim3(); //rotational 1
   init_tim14(); //rotational 2
      
   init_usart5(); //also enables USART3_8  //good
   init_tim6();                            //good
   init_usart1(); //linear 1               //good
   init_usart3(); //linear 2
   //nano_wait(50000);
   tic_exit_safe_start(USART1); //linear 1 //good
   tic_exit_safe_start(USART3); //linear 2
   //nano_wait(50000);
   nano_wait(10000);
   tic_energize(USART1); //linear 1        //good
   tic_energize(USART3); //linear 2 
   nano_wait(100000);

   //tic_set_target_position(USART1, -5000);
   //tic_set_target_position(USART3, -5000);
   //nano_wait(100000000);
   
   //center_rods();

   /*
   spin_motor1(-10000000); //rotational motor with lin motor 2 (2)
   spin_motor2(-10000000); //rotational motor with lin motor 1 (3)
   //tic_set_target_position(USART1, -5000);
   //tic_set_target_position(USART3, -5000);
   
   
   tic_set_target_position(USART1, -5000); //[5000, -4350]!!! //closest lin motor (0)
   tic_set_target_position(USART3, -5000); 
   nano_wait(100000000);
   tic_exit_safe_start(USART1); //linear 1 //good
   tic_exit_safe_start(USART3);
   nano_wait(10000);
   tic_set_target_position(USART1, 5000); //closest lin motor (0)
   tic_set_target_position(USART3, 5000);
   nano_wait(100000000);
   tic_exit_safe_start(USART1); //linear 1 //good
   tic_exit_safe_start(USART3);
   nano_wait(10000);
   tic_set_target_position(USART1, -5000); //[5000, -4350]!!! //closest lin motor (0)
   tic_set_target_position(USART3, -5000);
   nano_wait(100000000);
   tic_exit_safe_start(USART1); //linear 1 //good
   tic_exit_safe_start(USART3);
   nano_wait(10000);
   tic_set_target_position(USART1, 5000); //closest lin motor (0)
   tic_set_target_position(USART3, 5000);
   nano_wait(100000000);
   tic_exit_safe_start(USART1); //linear 1 //good
   tic_exit_safe_start(USART3);
   nano_wait(10000);
   tic_set_target_position(USART1, -5000); //[5000, -4350]!!! //closest lin motor (0)
   tic_set_target_position(USART3, -5000);
   nano_wait(100000000);
   tic_exit_safe_start(USART1); //linear 1 //good
   tic_exit_safe_start(USART3);
   nano_wait(10000);
   tic_set_target_position(USART1, 5000); //closest lin motor (0)
   tic_set_target_position(USART3, 5000);
   nano_wait(100000000);
   tic_exit_safe_start(USART1); //linear 1 //good
   tic_exit_safe_start(USART3);
   nano_wait(10000);
   tic_set_target_position(USART1, -5000); //[5000, -4350]!!! //closest lin motor (0)
   tic_set_target_position(USART3, -5000);
   nano_wait(100000000);
   tic_exit_safe_start(USART1); //linear 1 //good
   tic_exit_safe_start(USART3);
   nano_wait(10000);
   */
   

   //tic_set_target_position(USART1, 5000); //[5000, -4350]!!! //closest lin motor (0)
   //tic_set_target_position(USART3, 5000); //furthest lin motor (1)

   //tic_set_target_position(USART1, -4350); //closest lin motor (0)
   //tic_set_target_position(USART3, -4350); //furthest lin motor (1)

   //tic_set_target_position(USART1, 0); //closest lin motor (0)
   //tic_set_target_position(USART3, 0); //furthest lin motor (1)


   //uint8_t cmd[2] = {0xA2, 0x21};   //Get operation state
   //usart_send_array(USART1, cmd, 2);

   //PB0, PB1, PB2
   
   return EXIT_SUCCESS;
}
