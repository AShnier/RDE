/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Adam Shnier
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

// https://github.com/ThingPulse/esp8266-oled-ssd1306/ (accessed May2024)
//   Font generator (not used) http://oleddisplay.squix.ch/ (accessed May2024)
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"'
#include "QuickStats.h"
QuickStats stats; //initialize an instance of this class
#include <EEPROM.h>
#include <PID_v1.h>

// =============================
// Pin assignment
// =============================
 // Boot fails is pin 12 pulled high
 // GPIO 0 must be low to enter boot
 // GPIO 2 and 12 must be low to during boot
 // GPIO 5 and 15 must be high during boot
 // Pins 1,3,5,6or11,14,15 are high during boot
 // https://randomnerdtutorials.com/esp32-pinout-reference-gpios/ (accessed May2024)
 // Rotary encoder pins [Gnd, 3V3, SW, DT, CLK]
const byte rotpins[5] = {15,13,12,14,2};
const byte rot_sw = rotpins[2];
const byte tachPin = 25;   // select the input pin for tach
const byte pwmPin = 0;
const byte pwmChannel = 0;
const int pwmMin = 1700; // Motor specific
const int pwmMax = 8191; // 13-bit = {0:8191}
// =============================

// =============================
// Rotary encoder variables
// =============================
bool clkState = true;
bool dtState = true;
bool clkLastState = clkState;
int REdir = 1;
int REdir2 = 1; 
byte pos = 0;
// =============================

// =============================
// Tachometer and speed variables
// =============================
bool bTrig = 0; // Marks that the tachometer interval value has been updated
unsigned long tachOld = micros();
unsigned long tDisp = millis();
float rpms[8] = {0};
unsigned long c0_temp = 100000;// used to check for a non zero value before updating "c"
unsigned long c0 = 100000; 
float cTotal = 600000;
double pwm = 0;
int rpm = 0;
double rpm_med = 0;
// =============================

// =============================
// Tachometer and speed config variables
// =============================
int fins = 4;
int minRPM = 160, maxRPM = 4100;
int tISRmax = round(60000000/(fins*minRPM));
int tISRmin = round(60000000/(fins*maxRPM));

int loc_pwm = 0; // EEPROM address
// =============================

bool bRun = false;

int temp = 0; // An integer placeholder/temp value

// Initialize the OLED display using Wire library
SSD1306  display(0x3c, 5, 4);

// Enable or disable PID control
// https://web.archive.org/web/20221205222521/https://playground.arduino.cc/Code/PIDLibrary/
bool bPID = 1;
bool bAdjust = 0;
unsigned long tAdjust = millis();
double setpoint = 1600;

// myPID(... p, i, d, DIRECT);
PID myPID(&rpm_med, &pwm, &setpoint, 0.5, 2, 0, DIRECT);

// ============================
// Tachometer
// ============================
void IRAM_ATTR IRsense(){
    // It's best to avoid floating point operations in an ISR
    // An interrupt should be a quick to process as possible, digitalRead is effective yet not ideal in this case.
    // digitalRead takes approx. 0.17 microseconds according to https://forum.arduino.cc/t/microcontroller-i-o-adc-benchmarks/315304/33 (accessed 20May2024)
  if (digitalRead(tachPin)) { // Prevents trigger on falling https://github.com/espressif/arduino-esp32/issues/1111 (accessed 20May2024)
    c0_temp = (micros() - tachOld);
    if (c0_temp > tISRmin){ // to prevents double readings
      c0 = c0_temp;
      tachOld = micros();

      bTrig = 1;
    
    // ets_printf is an ISR compatible print statement. I does not round floats %d for int's, %f for floats
    //ets_printf("ISR interval (us): %d\n", c0);

    } 
  }
} // Close: IRsense
// =============================

void setup() {
  // Initialise Display
  display.init();
  //display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);
  
  // =============================
  // Initilise EEPROM
  if (!EEPROM.begin(512)){
    ets_printf("EEPROM failed to initialise\n");
  } else {
    ets_printf("EEPROM initialised\n");
  }

  // =============================
  // Assign pins and pin functions
  // =============================
  pinMode(tachPin, INPUT);
  pinMode(rotpins[0], OUTPUT);
  digitalWrite(rotpins[0], LOW);
  pinMode(rotpins[1], OUTPUT);
  digitalWrite(rotpins[1], HIGH);
  pinMode(rotpins[2], INPUT_PULLUP); // same as rot_sw
  pinMode(rotpins[3], INPUT_PULLUP);
  pinMode(rotpins[4], INPUT_PULLUP);
  
   // pin 0 must be low to boot
  ledcSetup(pwmChannel, 9000, 13); // Channel, freq, resolution
  ledcAttachPin(pwmPin, pwmChannel);

  attachInterrupt(digitalPinToInterrupt(tachPin),IRsense,RISING);  //  function for creating external interrupts at pin GPIO6 on Rising (LOW to HIGH)
  // =============================

  temp = EEPROM.get(loc_pwm, temp);
  if ((temp >= 0) && (temp <= pwmMax)){
    pwm = temp;
  }
  
  myPID.SetOutputLimits(pwmMin, pwmMax);
  myPID.SetMode(MANUAL);

  //Serial.begin(115200);
  
  ets_printf("No. fins: %d\n",fins);
  ets_printf("minRPM: %d\n",minRPM);
  ets_printf("maxRPM: %d\n",maxRPM);
  ets_printf("tISRmin: %d\n",tISRmin);
  ets_printf("tISRmax: %d\n",tISRmax);
}

void loop() {
  //delay(1000);

  rotary();

  // =============================
  // Push button (start and stop) 
  // ============================= 
  if (!digitalRead(rot_sw) && ((millis() - tAdjust) > 300)) {
    delay(10); // check if pressed for > 10 milliseconds
    if (!digitalRead(rot_sw)) {
      bRun = not(bRun);
      ets_printf("bRun: %d\n",(int) bRun);

      // used to delay the start of PID
      bAdjust - true;
      tAdjust = millis();

      if (!bPID || (abs(rpm_med - setpoint) < 5)){
        EEPROM.put(loc_pwm, (int) pwm);
        EEPROM.commit(); // needed to complete the eeprom put statement
        ets_printf("pwm to EEPROM: %d\n",(int) pwm);
      }
    }
  } // End: push button (rot_sw)
  // =============================

  // =============================
  // Calculate RPM
  // =============================
    // The actual work for the ISR
  if (bTrig && (c0 < tISRmax)){
    bTrig = 0;
    // RPM = 60s / time per rev = 60s / (fins*interval)
    rpm = round(60000000/(fins*c0));
    for(byte m = 7; m > 0; m = m - 1){
      rpms[m] = rpms[m-1];
    } // Close: for loop
    rpms[0] = rpm;
    //rpm_med = stats.median(rpms, 8);
    rpm_med = stats.average(rpms, 8);

    ets_printf("%ld %d %d\n", millis(),rpm, (int) pwm); 
  }

  // =============================
  // Set PWM for speed
  // =============================
  if (bRun) {
    // After the rotary encoder or rot_sw is used wait 2 second before starting PID
    if (bAdjust){
      if (millis() - tAdjust < 2000) {
        myPID.SetMode(MANUAL);
      } else if (bPID){
        setpoint = round(rpm_med/100) * 100;
        myPID.SetMode(AUTOMATIC);
        bAdjust = false; // This line will not run with bPID == false
      }
    } else {
      myPID.Compute(); // This does nothing if in MANUAL mode.
    } // End: if (bAdjust)
    // The PID package requires the pwm (output value) as a double, round is used before passing it to ledcWrite
    ledcWrite(pwmChannel, round(pwm));
  } else {
    ledcWrite(pwmChannel, 0);
  } // End: if (bRun)
  // =============================

  // =============================
  // Display
  // =============================
  if ((millis()-tDisp) > 300){
    
    tDisp = millis();

    // Check if last tachometer interval was slower than rpm min limit
    if ((micros() - tachOld) > tISRmax){
      rpm = 0;
      rpm_med = 0;
    }

    display.clear();

    display.setColor(WHITE);

    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "bRun: "+String(bRun));

    display.setTextAlignment(TEXT_ALIGN_CENTER);
    
    display.drawString(64, 54, "PWM: "+String((int)pwm));  
    display.setFont(ArialMT_Plain_24);
    display.drawString(64, 24, String((int)rpm_med)+" RPM");

    display.display();
    // =============================

  }
} // End: main loop

// ============================
// Rotary encoder
// ============================
void rotary() {
    clkState = digitalRead(rotpins[4]);
    dtState = digitalRead(rotpins[3]);
    if (clkState != clkLastState){
      if (dtState != clkState){
          // clockwise
        REdir = 1;
      } else {
          // anticlock
        REdir = -1;
      }
      if ((clkState == false)){//&&(REdir2 == REdir)){
        switch (pos){
          case 0:
            temp = pwm + (REdir*100);
            if ((temp >= pwmMin) && (temp <= pwmMax)){
              pwm = temp;
            } else if (temp < pwmMin){
              pwm = pwmMin;
            } else if (temp > pwmMax){
              pwm = pwmMax;
            } 
            break;
        }
        bAdjust = 1;
        tAdjust = millis();
      }
      REdir2 = REdir;
      delay(10); // millisecond delay to prevent double counts
    }
    clkLastState = clkState;
}
