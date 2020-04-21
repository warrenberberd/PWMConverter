/*
    This project is for convertir PWM Signal from an RC Remote controll receiver (Ex : FRSKY X8R)
    wich is coded in PWM with a duty cycle of 10% (1->2ms value range over  18ms), to PWM signal
    with a duty cycle range of 100% (0->100%)

    The main usage is for replacing RF receiver, from a basic RC CAR, with a FRSKY module, 
    to control the car with a TARANIS remote control.

    With a ESP8266, gave me a 4 channels converter (4 in/4 out)
    
    Copyright (C) 2020 Alban Ponche
    https://github.com/warrenberberd/PWMConverter

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <Arduino.h>

// For ESP8266 (ESP12-E)
/*#define CHAN_1  D0
#define CHAN_2  D5
#define CHAN_3  D6
#define CHAN_4  D7
#define OUT_1  D1
#define OUT_2  D2
#define OUT_3  D3
#define OUT_4  D4
*/

// For ESP-M3
#define CHAN_1  2
#define CHAN_2  0
#define CHAN_3  4
#define CHAN_4  5
#define OUT_1  15
#define OUT_2  13
#define OUT_3  12
#define OUT_4  14

unsigned long DURATION_1; // Last seen Duration for CHAN_1
unsigned long DURATION_2; // Last seen Duration for CHAN_2
unsigned long DURATION_3; // Last seen Duration for CHAN_3
unsigned long DURATION_4; // Last seen Duration for CHAN_4

// PWM Signal from Remote RC is between 1000 us to 2000us
unsigned long PWM_MIN_DURATION=1000;
unsigned long PWM_MAX_DURATION=2000;

unsigned long PWM_OUTPUT_FREQ=1000; // To change PWM Frequency (default: 1kHz)

// Working variable
unsigned long PWM_RANGE;

unsigned long RANGE_DURATION1;
unsigned long RANGE_DURATION2;
unsigned long RANGE_DURATION3;
unsigned long RANGE_DURATION4;

void setup() {
  Serial.setDebugOutput(false);

  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial) ; // wait for Arduino Serial Monitor to open

    Serial.println("");
    delay(1000);

    
    Serial.setDebugOutput(true);
  #endif

  pinMode(CHAN_1, INPUT);
  pinMode(CHAN_2, INPUT);
  pinMode(CHAN_3, INPUT);
  pinMode(CHAN_4, INPUT);

  pinMode(OUT_1, OUTPUT);
  pinMode(OUT_2, OUTPUT);
  pinMode(OUT_3, OUTPUT);
  pinMode(OUT_4, OUTPUT);

  // Calculation PWM Range
  PWM_RANGE=PWM_MAX_DURATION-PWM_MIN_DURATION;

  analogWriteRange(PWM_RANGE);         // Setting range from 0 to 1000
  analogWriteFreq(PWM_OUTPUT_FREQ); // To change PWM Frequency (default: 1kHz)
}

void loop() {
  #ifdef DEBUG
    //Serial.println("Acquire PWM input...");
  #endif
  // Getting Raw duration from PWN INPUT (RC controller)
  DURATION_1 = pulseIn(CHAN_1, HIGH);
  DURATION_2 = pulseIn(CHAN_2, HIGH);
  DURATION_3 = pulseIn(CHAN_3, HIGH);
  DURATION_4 = pulseIn(CHAN_4, HIGH);

  #ifdef DEBUG
    Serial.printf("    IN PWM value : %lu\t%lu\t%lu\t%lu\n",DURATION_1,DURATION_2,DURATION_3,DURATION_4);
  #endif

  // Shift range from 1000-2000 to 0-1000
  if(DURATION_1>PWM_MIN_DURATION) RANGE_DURATION1=DURATION_1-PWM_MIN_DURATION;
  if(DURATION_2>PWM_MIN_DURATION) RANGE_DURATION2=DURATION_2-PWM_MIN_DURATION;
  if(DURATION_3>PWM_MIN_DURATION) RANGE_DURATION3=DURATION_3-PWM_MIN_DURATION;
  if(DURATION_4>PWM_MIN_DURATION) RANGE_DURATION4=DURATION_4-PWM_MIN_DURATION;

  // Outputing the Full scale PWM Output
  analogWrite(OUT_1,RANGE_DURATION1);
  analogWrite(OUT_2,RANGE_DURATION2);
  analogWrite(OUT_3,RANGE_DURATION3);
  analogWrite(OUT_4,RANGE_DURATION4);

  #ifdef DEBUG
    Serial.printf("   OUT PWM value : %lu\t%lu\t%lu\t%lu\n",RANGE_DURATION1,RANGE_DURATION2,RANGE_DURATION3,RANGE_DURATION4);
  #endif

  optimistic_yield(1000);
}