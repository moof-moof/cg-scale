/*************************************************************************************************
 *  Firmware for a center-of-gravity balance for medium-sized straight-winged aircraft models.
 *  Original concept by Olav Kallhovd 2016-2017, extended by Alain Desandre 2018.
 *  Adapted for a simplified hardware design by Martin Bergman 2018.
 * 
 *  Main instrumentaion components:
 *    1 pc load sensor YZC-133 2kg (front)
 *    1 pc load sensor YZC-133 3kg (rear)
 *    2 pcs HX711 ADC, one for each load sensor (128bit resolution)
 *    1 pc 16*2 HD44780 LCD using I2C interface
 *    1 pc Arduino (any) for driving the scales and the display
 * 
 *    (Maximum model mass with above sensors: 4 to 4.5kg depending on CG location)
 *************************************************************************************************/

#include <LiquidCrystal_I2C.h>

#include <HX711_ADC.h> /* https://github.com/olkal/HX711_ADC can be installed from the library manager.
* Number of samples and some filtering settings can be adjusted in the HX711_ADC.h library file.
* The best RATE setting is usually 10SPS, see HX711 data sheet (HX711 pin 15, which can usually be set by
* a solder jumper on the HX711 module). RATE 80SPS will also work fine, but conversions will be more noisy,
* so consider increasing number of samples in HX711_ADC.h.
*/

// 1602 LCD constructor:
LiquidCrystal_I2C lcd(0x27,16,2);  // Set the LCD address to 0x3F or 0x27 for a 16 chars and 2 line display.

// HX711 constructor:
HX711_ADC LoadCell_1(A2, A3); //HX711 pins front sensor (DOUT, PD_SCK).
HX711_ADC LoadCell_2(A0, A1); //HX711 pins rear sensor (DOUT, PD_SCK).


uint8_t aring[8] = {0x06,0x06,0x00,0x0E,0x01,0x0F,0x11,0x0F};   // Custom LCD character definition.
uint8_t ledPin = 13;            // The onboard led. Flashing indicates activity.
uint8_t extraLED = 3;
boolean ledState;
char toLCD[20];
uint8_t displayMode;            // One of two possible states: 0 for serial (monitor), 2 for I²C (LCD display).
int32_t t1;
int32_t t2;

const int printInterval = 500;  // LCD/Serial refresh interval.



/// CONFIGURATION SECTIOM:

// Set the dimensional calibration values:
const int32_t WingPegDist = 1342;   // (Mean of 1335/1350) calibration value in 1/10mm,
                                    // projected distance between wing support points.
const int32_t LEstopperDist = 225;  // Calibration value in 1/10mm, projected dist from front wing support
                                    // point to LE (stopper pin).
// Set scales calibration values:
const float ldcell_1_calfactor = 933.0;     // User set calibration factor load cell front (float).
const float ldcell_2_calfactor = 1012.0;    // User set calibration factor load cell rear (float).

const int32_t stabilisingtime = 3000; // Tare precision can be improved by adding a few seconds of stabilising time.

const int32_t CGoffset = ((WingPegDist / 2) + LEstopperDist) * 10;



/// YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY



void setup() {


    displayMode = 2;                // 0 => Serial-Console (PC);  2 => I2C-LCD

    pinMode(ledPin, OUTPUT);        // Standard onboard LED.
    pinMode(extraLED, OUTPUT);      // More conspicious LED (remotely positioned).
    digitalWrite(ledPin, HIGH);
    digitalWrite(extraLED, HIGH);
    
    Serial.begin(9600);             // Open serial monitor for possible debugging.
    
    Serial.print("CG balance monitor says: \n");

    if (displayMode == 0) {         // If outputting to the serial terminal.
        Serial.println();
        Serial.println("Wait for stabilising and tare...");
    }


    if (displayMode == 2){          // If using the i2c display
        Serial.println("Initializing I2C-Display (displayMode == 2), although your are still reading this as serial!");
        
        lcd.init();
        lcd.backlight();
        delay(3000);                // Improves stabilisation?
        
        lcd.createChar(0, aring);   // Generates the lower-case aring-character (å).
        
        lcd.setCursor(0, 0);
            lcd.print("Tyngdpunktsv");
            lcd.write(uint8_t(0));   // å ("null" needs to be casted to zero).
            lcd.print("gen");
        lcd.setCursor(0, 1);
            lcd.write(225);          // ä
            lcd.print("r strax klar!");
    }

    // HX711 init and calibration factor setting:
    LoadCell_1.begin();
    LoadCell_2.begin();
    
    uint8_t loadcell_1_rdy = 0;
    uint8_t loadcell_2_rdy = 0;

    // Do startup, stabilisation and tare, on both modules simultaneously.
    while ((loadcell_1_rdy + loadcell_2_rdy) < 2) { 
        if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilisingtime);
        if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilisingtime);
    }
    LoadCell_1.setCalFactor(ldcell_1_calfactor); // Set front calibration factor
    LoadCell_2.setCalFactor(ldcell_2_calfactor); // Set rear calibration factor

    delay(3000);                     // Some free time to "get balanced" doesn't hurt.
}


/// YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY



void loop() {
    // Library function update() should be called at least as often as HX711 sample rate; >10Hz@10SPS,
    // >80Hz@80SPS. Longer delay will reduce effective sample rate -- be careful with delay() in loop().
    LoadCell_1.update();
    LoadCell_2.update();
    
    // Calculate CG and update serial/LCD
    if (t1 < millis()) {
        t1 = millis() + printInterval;      // General refresh timer
        float a = LoadCell_1.getData();
        float b = LoadCell_2.getData();
        int32_t weightAvr[3];
        float CGratio;
        int32_t CG;
        weightAvr[0] = a * 100;
        weightAvr[1] = b * 100;
        int32_t weightTot = weightAvr[0] + weightAvr[1];

        if (weightAvr[0] > 500 && weightAvr[1] > 500) {
            int32_t a = weightAvr[1] / 10;
            int32_t b = weightAvr[0] / 10;
            CGratio = (((a * 10000) / (a + b))); 
            CG = ((((WingPegDist) * CGratio) / 1000) - ((WingPegDist * 10) / 2) + CGoffset);
        }
        else {
            CG = 0;
        }

        if (displayMode == 0) {              // if displayMode = 0: print result to serial terminal
            for (uint8_t a = 0; a < 2; a++) {
                Serial.print("weight_LdCell_");
                Serial.print(a + 1);
                Serial.print(": ");
                int32_t i = weightAvr[a];
                if (i < 0) {
                    Serial.print('-');
                    i = ~weightAvr[a];
                }
                Serial.print(i / 100);
                Serial.print('.');
                if ((i % 100) < 10) {
                    Serial.print("0");
                }
                Serial.print(i % 100);
                Serial.print("      ");
            }
            Serial.print("CG:");
            Serial.print(CG / 100);
            Serial.print('.');
            Serial.println(CG % 100);
        }

        else if (displayMode == 2) {        // if displayMode = 2: print to I2C LCD

            lcd.clear();

            lcd.setCursor(0, 0);            // First line: Weight
            if (weightTot < 0 && weightTot >= - 100)
                weightTot = 0;
            if (weightTot < -100)
                lcd.print("Wt: Error!");
            else {
                sprintf(toLCD, "Wt: %ug", weightTot/100);
                lcd.print(toLCD);
            }

            lcd.setCursor(0, 1);            // Second line: CG
            if (CG == 0)
            lcd.print("CG: Out of range");
            else {
                sprintf(toLCD, "CG: %ld.%ldmm", CG/100, CG%100);
                lcd.print(toLCD);
            }

        }
    }
    flashLED();
}


/// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII



void flashLED() {                               // Blink timer routine
    if (t2 < millis()) {
        if (ledState) {
            t2 = millis() + 2000;
            ledState = 0;
        }
        else {
            t2 = millis() + 100;
            ledState = 1;
        }
        digitalWrite(ledPin, ledState);
        digitalWrite(extraLED, ledState);
    }
}
