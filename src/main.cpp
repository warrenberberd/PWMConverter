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


// Custom variables
unsigned long DURATION_1; // Last seen Duration for CHAN_1
unsigned long DURATION_2; // Last seen Duration for CHAN_2
unsigned long DURATION_3; // Last seen Duration for CHAN_3
unsigned long DURATION_4; // Last seen Duration for CHAN_4

// PWM Signal from Remote RC is between 1000 us to 2000us
unsigned long PWM_MIN_DURATION=1000;
unsigned long PWM_MAX_DURATION=2000;
unsigned long PWM_OLD_MAX_DURATION=2000;
unsigned long PWM_MAX_IGNORE=2016;        // Value greater than this will be ignore (Theoretically : 2000 is max)

unsigned long REAL_PWM_MIN_VALUE1; // The MIN value seen for channel1
unsigned long REAL_PWM_MIN_VALUE2; // The MIN value seen for channel2
unsigned long REAL_PWM_MIN_VALUE3; // The MIN value seen for channel3
unsigned long REAL_PWM_MIN_VALUE4; // The MIN value seen for channel4

unsigned long REAL_PWM_MAX_VALUE1; // The MAX value seen for channel1
unsigned long REAL_PWM_MAX_VALUE2; // The MAX value seen for channel2
unsigned long REAL_PWM_MAX_VALUE3; // The MAX value seen for channel3
unsigned long REAL_PWM_MAX_VALUE4; // The MAX value seen for channel4

unsigned long PWM_OUTPUT_FREQ=1000; // To change PWM Frequency (default: 1kHz)

// Working variable
unsigned long MAX_OUT_PWM_RANGE;

unsigned long RANGE_DURATION1=0;
unsigned long RANGE_DURATION2=0;
unsigned long RANGE_DURATION3=0;
unsigned long RANGE_DURATION4=0;

bool GO_FORWARD=false;
bool GO_BACKWARD=false;
bool GO_LEFT=false;
bool GO_RIGHT=false;

unsigned long ADVANCE_VALUE=0;
unsigned long ROTATION_VALUE=0;



// To auto-calibrate the values
void autoTrimValues(){
  // Recalculate REAL MIN value
  // The -100 is for the imprecision of PWM mesure
  if(REAL_PWM_MIN_VALUE1>DURATION_1 && DURATION_1>PWM_MIN_DURATION-100)  REAL_PWM_MIN_VALUE1=DURATION_1;
  if(REAL_PWM_MIN_VALUE2>DURATION_2 && DURATION_2>PWM_MIN_DURATION-100)  REAL_PWM_MIN_VALUE2=DURATION_2;
  if(REAL_PWM_MIN_VALUE3>DURATION_3 && DURATION_3>PWM_MIN_DURATION-100)  REAL_PWM_MIN_VALUE3=DURATION_3;
  if(REAL_PWM_MIN_VALUE4>DURATION_4 && DURATION_4>PWM_MIN_DURATION-100)  REAL_PWM_MIN_VALUE4=DURATION_4;

  // Shift range from 1000-2000 to 0-1000
  if(DURATION_1>REAL_PWM_MIN_VALUE1-100) RANGE_DURATION1=DURATION_1-REAL_PWM_MIN_VALUE1;
  if(DURATION_2>REAL_PWM_MIN_VALUE2-100) RANGE_DURATION2=DURATION_2-REAL_PWM_MIN_VALUE2;
  if(DURATION_3>REAL_PWM_MIN_VALUE3-100) RANGE_DURATION3=DURATION_3-REAL_PWM_MIN_VALUE3;
  if(DURATION_4>REAL_PWM_MIN_VALUE4-100) RANGE_DURATION4=DURATION_4-REAL_PWM_MIN_VALUE4;

  // Recalculate REAL MAX value
  if(DURATION_1>REAL_PWM_MAX_VALUE1 && DURATION_1<PWM_MAX_DURATION+100) REAL_PWM_MAX_VALUE1=DURATION_1;
  if(DURATION_2>REAL_PWM_MAX_VALUE2 && DURATION_2<PWM_MAX_DURATION+100) REAL_PWM_MAX_VALUE2=DURATION_2;
  if(DURATION_3>REAL_PWM_MAX_VALUE3 && DURATION_3<PWM_MAX_DURATION+100) REAL_PWM_MAX_VALUE3=DURATION_3;
  if(DURATION_4>REAL_PWM_MAX_VALUE4 && DURATION_4<PWM_MAX_DURATION+100) REAL_PWM_MAX_VALUE4=DURATION_4;

  // Check if one MAX value exceed GLOBAL MAX
  PWM_OLD_MAX_DURATION=PWM_MAX_DURATION;  // Backuping the MAX value
  if(REAL_PWM_MAX_VALUE1>PWM_MAX_DURATION) PWM_MAX_DURATION=REAL_PWM_MAX_VALUE1;
  if(REAL_PWM_MAX_VALUE2>PWM_MAX_DURATION) PWM_MAX_DURATION=REAL_PWM_MAX_VALUE2;
  if(REAL_PWM_MAX_VALUE3>PWM_MAX_DURATION) PWM_MAX_DURATION=REAL_PWM_MAX_VALUE3;
  if(REAL_PWM_MAX_VALUE4>PWM_MAX_DURATION) PWM_MAX_DURATION=REAL_PWM_MAX_VALUE4;

  // if MAX value have changed, we redefine the RANGE
  if(PWM_OLD_MAX_DURATION<PWM_MAX_DURATION){
    #ifdef DEBUG
      Serial.printf("New MAX is : %lu\n",PWM_MAX_DURATION);
    #endif
    // Calculation PWM Default Range
    MAX_OUT_PWM_RANGE=PWM_MAX_DURATION-PWM_MIN_DURATION;
    analogWriteRange(MAX_OUT_PWM_RANGE);         // Setting range from 0 to 1000
  }
}

// Acquire input
void acquireInputs(){
  #ifdef DEBUG
    //Serial.println("Acquire PWM input...");
  #endif
  // Getting Raw duration from PWN INPUT (RC controller)
  DURATION_1 = pulseIn(CHAN_1, HIGH);
  DURATION_2 = pulseIn(CHAN_2, HIGH);
  DURATION_3 = pulseIn(CHAN_3, HIGH);
  DURATION_4 = pulseIn(CHAN_4, HIGH);

  // Protection
  if(DURATION_1>PWM_MAX_IGNORE) DURATION_1=REAL_PWM_MAX_VALUE1;
  if(DURATION_2>PWM_MAX_IGNORE) DURATION_2=REAL_PWM_MAX_VALUE2;
  if(DURATION_3>PWM_MAX_IGNORE) DURATION_3=REAL_PWM_MAX_VALUE3;
  if(DURATION_4>PWM_MAX_IGNORE) DURATION_4=REAL_PWM_MAX_VALUE4;

  // Auto-calibration
  autoTrimValues();

}

// Outputing Full Scale PWM Values
void output(){

  // Protection
  if(RANGE_DURATION1>MAX_OUT_PWM_RANGE) RANGE_DURATION1=MAX_OUT_PWM_RANGE;
  if(RANGE_DURATION2>MAX_OUT_PWM_RANGE) RANGE_DURATION2=MAX_OUT_PWM_RANGE;
  if(RANGE_DURATION3>MAX_OUT_PWM_RANGE) RANGE_DURATION3=MAX_OUT_PWM_RANGE;
  if(RANGE_DURATION4>MAX_OUT_PWM_RANGE) RANGE_DURATION4=MAX_OUT_PWM_RANGE;


  // Outputing the Full scale PWM Output
  analogWrite(OUT_1,RANGE_DURATION1);
  analogWrite(OUT_2,RANGE_DURATION2);
  analogWrite(OUT_3,RANGE_DURATION3);
  analogWrite(OUT_4,RANGE_DURATION4);

  #ifdef DEBUG
    Serial.printf("    IN PWM value : %lu\t%lu\t%lu\t%lu\t\t",DURATION_1,DURATION_2,DURATION_3,DURATION_4);
  #endif

  #ifdef DEBUG
    Serial.printf("   OUT PWM value : %lu\t%lu\t%lu\t%lu\n",RANGE_DURATION1,RANGE_DURATION2,RANGE_DURATION3,RANGE_DURATION4);
    //Serial.printf("RANGE is : %lu\n",MAX_OUT_PWM_RANGE);
  #endif
}

// To process the TILT stick
unsigned long shiftForwardValue(unsigned long SPEED_VALUE,unsigned long TILT_VALUE){
  // The ADVANCE value take the value of SPEED
  unsigned long ADVANCE_VALUE=SPEED_VALUE;

  unsigned long TILT_TRIGGER=20;
  unsigned long MIDDLE_TILT=(REAL_PWM_MAX_VALUE3-REAL_PWM_MIN_VALUE3)/2;//+REAL_PWM_MIN_VALUE3;

  GO_FORWARD=false;
  GO_BACKWARD=false;
  unsigned long REL_TILT_VALUE=0;
  unsigned long REL_TILT_MAX=(REAL_PWM_MAX_VALUE3-REAL_PWM_MIN_VALUE3)/2;
  if(TILT_VALUE>MIDDLE_TILT+TILT_TRIGGER){
    GO_FORWARD=true;

    REL_TILT_VALUE=TILT_VALUE-MIDDLE_TILT;  // Get the Tilt Value referenced to Middle

    #ifdef DEBUG
      //Serial.print(" GO FORWARD !");
      //Serial.printf("REL_TILT_VALUE=%lu\t\tREL_TILT_MAX=%lu\n",REL_TILT_VALUE,REL_TILT_MAX);
    #endif

    // then we scale with TILT value
    ADVANCE_VALUE=ADVANCE_VALUE*REL_TILT_VALUE/REL_TILT_MAX;

    // We set the SPEED value
    RANGE_DURATION1=RANGE_DURATION3=ADVANCE_VALUE;
    
  }else if(TILT_VALUE<MIDDLE_TILT-TILT_TRIGGER){
    GO_BACKWARD=true;

    REL_TILT_VALUE=MIDDLE_TILT-TILT_VALUE;  // Get the Tilt Value referenced to Middle

    #ifdef DEBUG
      //Serial.print(" GO BACKWARD ! ");
      //Serial.printf("REL_TILT_VALUE=%lu\t\tREL_TILT_MAX=%lu\n",REL_TILT_VALUE,REL_TILT_MAX);
    #endif

    // then we scale with TILT value
    ADVANCE_VALUE=ADVANCE_VALUE*REL_TILT_VALUE/REL_TILT_MAX;

    // We set the SPEED value
    RANGE_DURATION2=RANGE_DURATION4=ADVANCE_VALUE;
  }else{
    ADVANCE_VALUE=0;
  }

  return ADVANCE_VALUE;
}

// To process the PAN stick
unsigned long shiftRotationValue(unsigned long SPEED_VALUE,unsigned long PAN_VALUE,unsigned long ADVANCE_VALUE){
  // Output Mapping : 
  // OUT_1=WHEEL1_CW
  // OUT_2=WHEEL1_CCW
  // OUT_3=WHEEL2_CW
  // OUT_4=WHEEL2_CCW
  int WHEEL1_CW=0;
  int WHEEL1_CCW=0;
  int WHEEL2_CW=0;
  int WHEEL2_CCW=0;

  //unsigned long ROTATION_VALUE=SPEED_VALUE;

  unsigned long PAN_TRIGGER=20;
  unsigned long MIDDLE_PAN=(REAL_PWM_MAX_VALUE2-REAL_PWM_MIN_VALUE2)/2;//+REAL_PWM_MIN_VALUE2;

  GO_LEFT=false;
  GO_RIGHT=false;
  unsigned long REL_PAN_VALUE=0;
  unsigned long REL_PAN_MAX=(REAL_PWM_MAX_VALUE2-REAL_PWM_MIN_VALUE2)/2;

  if(PAN_VALUE>MIDDLE_PAN+PAN_TRIGGER){ // We Want to go Right
    GO_RIGHT=true;

    REL_PAN_VALUE=PAN_VALUE-MIDDLE_PAN;  // Get the PAN Value referenced to Middle
    ROTATION_VALUE=SPEED_VALUE*REL_PAN_VALUE/REL_PAN_MAX; // then we scale with PAN value
    //if(ROTATION_VALUE>ADVANCE_VALUE && ADVANCE_VALUE>0) ROTATION_VALUE=ADVANCE_VALUE;
    //if(ROTATION_VALUE<ADVANCE_VALUE && ROTATION_VALUE>0) ADVANCE_VALUE=ROTATION_VALUE;

    #ifdef DEBUG
      //Serial.print(" GO RIGHT !");
      //Serial.printf("REL_PAN_VALUE=%lu\t\REL_PAN_MAX=%lu\n",REL_PAN_VALUE,REL_PAN_MAX);
      //Serial.printf(" RotationValue: %lu\t",ROTATION_VALUE);
    #endif

    if(GO_FORWARD){        // GO_FORWARD AND GO_RIGHT
      WHEEL1_CW=ADVANCE_VALUE;
      if(ROTATION_VALUE>WHEEL1_CW) WHEEL1_CW=ROTATION_VALUE;
      WHEEL2_CW=ADVANCE_VALUE-ROTATION_VALUE;
      if(WHEEL2_CW<0) WHEEL2_CW=0;
      WHEEL2_CCW=ROTATION_VALUE-ADVANCE_VALUE;
      if(WHEEL2_CCW<0) WHEEL2_CCW=0;
    }else if(GO_BACKWARD){  // GO_BACKWARD AND GO_RIGHT
      WHEEL1_CCW=ADVANCE_VALUE;
      WHEEL2_CCW=ADVANCE_VALUE-ROTATION_VALUE;
      if(WHEEL2_CCW<0) WHEEL2_CCW=WHEEL2_CCW*-1;
      //WHEEL2_CW=ROTATION_VALUE-ADVANCE_VALUE;
      //if(WHEEL2_CW<0) WHEEL2_CW=0;
    }else{  // If Wheel1 dont move
      WHEEL1_CW+=ROTATION_VALUE;
      WHEEL2_CCW+=ROTATION_VALUE;
    }
  }else if(PAN_VALUE<MIDDLE_PAN-PAN_TRIGGER){ // We Want to go Left
    GO_LEFT=true;

    REL_PAN_VALUE=MIDDLE_PAN-PAN_VALUE;  // Get the PAN Value referenced to Middle
    ROTATION_VALUE=SPEED_VALUE*REL_PAN_VALUE/REL_PAN_MAX; // then we scale with PAN value
    //if(ROTATION_VALUE>ADVANCE_VALUE && ADVANCE_VALUE>0) ROTATION_VALUE=ADVANCE_VALUE;
    //if(ROTATION_VALUE<ADVANCE_VALUE && ROTATION_VALUE>0) ADVANCE_VALUE=ROTATION_VALUE;

    #ifdef DEBUG
      //Serial.print(" GO LEFT ! ");
      //Serial.printf("REL_PAN_VALUE=%lu\t\REL_PAN_MAX=%lu\n",REL_PAN_VALUE,REL_PAN_MAX);
      //Serial.printf(" RotationValue: %lu\t",ROTATION_VALUE);
    #endif

    if(GO_FORWARD){        // GO_FORWARD AND GO_LEFT
      WHEEL2_CW=ADVANCE_VALUE;
      if(ROTATION_VALUE>WHEEL2_CW) WHEEL2_CW=ROTATION_VALUE;
      WHEEL1_CW=ADVANCE_VALUE-ROTATION_VALUE;
      if(WHEEL1_CW<0) WHEEL1_CW=0;
      WHEEL1_CCW=ROTATION_VALUE-ADVANCE_VALUE;
      if(WHEEL1_CCW<0) WHEEL1_CCW=0;
      //WHEEL1_CCW=ROTATION_VALUE-ADVANCE_VALUE;
    }else if(GO_BACKWARD){  // GO_BACKWARD AND GO_LEFT
      WHEEL2_CCW=ADVANCE_VALUE;
      WHEEL1_CCW=ADVANCE_VALUE-ROTATION_VALUE;
      if(WHEEL1_CCW<0) WHEEL1_CCW=WHEEL1_CCW*-1;
      //WHEEL1_CW=ROTATION_VALUE-ADVANCE_VALUE;
      //if(WHEEL1_CW<0) WHEEL1_CW=0;
    }else{  // If Wheel1 dont move
      WHEEL1_CCW+=ROTATION_VALUE;
      WHEEL2_CW+=ROTATION_VALUE;
    }
  }else{  // No PAN (No Rotation)
    ROTATION_VALUE=0;
    //WHEEL1_CW=WHEEL1_CCW=WHEEL2_CW=WHEEL2_CCW=ROTATION_VALUE;
  }

  #ifdef DEBUG
    //Serial.printf("ROTATION_VALUE=%lu\t",ROTATION_VALUE);
  #endif

  // Output Mapping : 
  // OUT_1=WHEEL1_CW
  // OUT_2=WHEEL1_CCW
  // OUT_3=WHEEL2_CW
  // OUT_4=WHEEL2_CCW

  // If no rotation, dont touch anything
  if(ROTATION_VALUE<1) return ROTATION_VALUE;

  // If values are negative, inverse it
  //if(WHEEL1_CW<0)   WHEEL1_CW=WHEEL1_CW*-1;
  //if(WHEEL1_CCW<0)  WHEEL1_CCW=WHEEL1_CCW*-1;
  //if(WHEEL2_CW<0)   WHEEL2_CW=WHEEL2_CW*-1;
  //if(WHEEL2_CCW<0)  WHEEL2_CCW=WHEEL2_CCW*-1;

  RANGE_DURATION1=WHEEL1_CW;
  RANGE_DURATION2=WHEEL1_CCW;
  RANGE_DURATION3=WHEEL2_CW;
  RANGE_DURATION4=WHEEL2_CCW;

  return ROTATION_VALUE;
}

// Translating value for a Dual Wheel CAR Output
void shiftToDualWheelMode(){
  // Input Mapping : 
  // CHAN_1=SPEED
  // CHAN_2=LEFT/RIGHT       : PAN  (centered by default)
  // CHAN_3=FORWARD/BACKWARD : TILT (centered by default)
  // CHAN_4=NOT USE
  unsigned long TILT_VALUE=RANGE_DURATION3;
  //unsigned long TILT_VALUE_RAW=DURATION_3;
  unsigned long PAN_VALUE=RANGE_DURATION2;
  //unsigned long PAN_VALUE_RAW=DURATION_2;
  unsigned long SPEED_VALUE=RANGE_DURATION1;
  //unsigned long SPEED_VALUE_RAW=DURATION_1;

  // To say ALL wheel stop
  RANGE_DURATION1=0;
  RANGE_DURATION3=0;
  RANGE_DURATION2=0;
  RANGE_DURATION4=0;

  ADVANCE_VALUE=shiftForwardValue(SPEED_VALUE,TILT_VALUE);

  ROTATION_VALUE=shiftRotationValue(SPEED_VALUE,PAN_VALUE,ADVANCE_VALUE);


  // Output Mapping : 
  // OUT_1=WHEEL1_CW
  // OUT_2=WHEEL1_CCW
  // OUT_3=WHEEL2_CW
  // OUT_4=WHEEL2_CCW

}

void enableLED(){
  //digitalWrite(LED_BUILTIN,LOW);
  #ifdef DEBUG
    Serial.println("AUTOCAL : Waiting for trim value...");
  #endif
}

void disableLED(){
  //digitalWrite(LED_BUILTIN,HIGH);
  #ifdef DEBUG
    Serial.println("AUTOCAL :     DETECTED !");
  #endif

}

// Autocalibration of PWM MIN/MAX Value for remote, before use
void autocalRcRemote(){
  //pinMode(LED_BUILTIN,OUTPUT);

  // Wait for SPEED trimmer, to LOW
  enableLED();
  acquireInputs();
  while(DURATION_1>1000 || DURATION_1<900){
    optimistic_yield(1000);
    delay(15);
    acquireInputs();
  }
  disableLED();
  delay(1000);
  

  // Wait for SPEED trimmer, to HIGH
  enableLED();
  acquireInputs();
  while(DURATION_1<2000){
    optimistic_yield(1000);
    delay(15);
    acquireInputs();
  }
  disableLED();
  delay(1000);

  // Wait for PAN trimmer, to LEFT
  enableLED();
  acquireInputs();
  while(DURATION_2>1000 || DURATION_2<900){
    optimistic_yield(1000);
    delay(15);
    acquireInputs();
  }
  disableLED();
  delay(1000);

  // Wait for PAN trimmer, to RIGHT
  enableLED();
  acquireInputs();
  while(DURATION_2<2000){
    optimistic_yield(1000);
    delay(15);
    acquireInputs();
  }
  disableLED();
  delay(1000);

  // Wait for TILT trimmer, to LOW
  enableLED();
  acquireInputs();
  while(DURATION_3>1000 || DURATION_3<900){
    optimistic_yield(1000);
    delay(15);
    acquireInputs();
  }
  disableLED();
  delay(1000);

  // Wait for TILT trimmer, to HIGH
  enableLED();
  acquireInputs();
  while(DURATION_3<2000){
    optimistic_yield(1000);
    delay(15);
    acquireInputs();
  }
  disableLED();
  delay(1000);


}

// Main Setup
void setup() {
  

  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial) ; // wait for Arduino Serial Monitor to open

    Serial.setDebugOutput(false);

    Serial.println("");
    delay(1000);

    
    Serial.setDebugOutput(true);
  #endif

  pinMode(CHAN_1, INPUT);
  pinMode(CHAN_2, INPUT);
  pinMode(CHAN_3, INPUT);
  pinMode(CHAN_4, INPUT);

  // Auto calibration of RC Remote
  autocalRcRemote();

  pinMode(OUT_1, OUTPUT);
  pinMode(OUT_2, OUTPUT);
  pinMode(OUT_3, OUTPUT);
  pinMode(OUT_4, OUTPUT);

  // Force early output zero
  output();

  
  

  // By default, we presume that the MIN value for each channel is PWM_MIN_DURATION
  REAL_PWM_MIN_VALUE1=PWM_MIN_DURATION;
  REAL_PWM_MIN_VALUE2=PWM_MIN_DURATION;
  REAL_PWM_MIN_VALUE3=PWM_MIN_DURATION;
  REAL_PWM_MIN_VALUE4=PWM_MIN_DURATION;
  // Same thing for MAX
  REAL_PWM_MAX_VALUE1=PWM_MAX_DURATION;
  REAL_PWM_MAX_VALUE2=PWM_MAX_DURATION;
  REAL_PWM_MAX_VALUE3=PWM_MAX_DURATION;
  REAL_PWM_MAX_VALUE4=PWM_MAX_DURATION;

  // Calculation PWM Default Range
  MAX_OUT_PWM_RANGE=PWM_MAX_DURATION-PWM_MIN_DURATION;
  analogWriteRange(MAX_OUT_PWM_RANGE);         // Setting range from 0 to 1000

  analogWriteFreq(PWM_OUTPUT_FREQ); // To change PWM Frequency (default: 1kHz)
}

// Main loop
void loop() {
  optimistic_yield(1000);

  // Acquire input
  acquireInputs();

  unsigned long DETECTION_TRIGGER=800;
  if(DURATION_1<DETECTION_TRIGGER || DURATION_2<DETECTION_TRIGGER || DURATION_3<DETECTION_TRIGGER || DURATION_4<DETECTION_TRIGGER){
    #ifdef DEBUG
      Serial.println("   No Data. Ignore.");
    #endif
    return;
  }

  // Translating value for DualWheel CAR
  shiftToDualWheelMode();

  // Outputing values
  output();

  
}