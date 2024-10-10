#include <Wire.h>
#include "Adafruit_seesaw.h"

// Joystick I2C settings
#define JOY_ADDR 0x52  // Define Joystick I2C address
#define SDA_PIN 2      // Set SDA to GPIO2
#define SCL_PIN 3      // Set SCL to GPIO3

// Gamepad button definitions
#define BUTTON_X         6
#define BUTTON_Y         2
#define BUTTON_A         5
#define BUTTON_B         1
#define BUTTON_SELECT    0
#define BUTTON_START    16

uint32_t button_mask = (1UL << BUTTON_X) | (1UL << BUTTON_Y) | (1UL << BUTTON_START) |
                       (1UL << BUTTON_A) | (1UL << BUTTON_B) | (1UL << BUTTON_SELECT);

// Global variables
Adafruit_seesaw ss;
char data[100];
int last_x = 0, last_y = 0;
static uint8_t x_data, y_data, button_data;

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while(!Serial) {
        delay(10);
    }
    Serial.println("Initializing...");

    // Initialize I2C for joystick (set SDA and SCL pins)
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);
    Wire.setSDA(SDA_PIN);
    Wire.setSCL(SCL_PIN);
    Wire.begin();
    Serial.println("Initialized GPIO2 as SDA and GPIO3 as SCL");

    // Initialize the Adafruit seesaw gamepad
    if (!ss.begin(0x50)) {
        Serial.println("ERROR! seesaw not found");
        while(1) delay(1);
    }
    Serial.println("seesaw started");
    uint32_t version = ((ss.getVersion() >> 16) & 0xFFFF);
    if (version != 5743) {
        Serial.print("Wrong firmware loaded? ");
        Serial.println(version);
        while(1) delay(10);
    }
    Serial.println("Found Product 5743");

    // Configure button inputs on the seesaw
    ss.pinModeBulk(button_mask, INPUT_PULLUP);
    ss.setGPIOInterrupts(button_mask, 1);
}

void loop() {
    delay(10);  // Delay to slow serial output

    // ********** Handle Joystick I2C communication **********
    // Request 3 bytes from the joystick (slave device)
    Wire.requestFrom(JOY_ADDR, 3);  
    if (Wire.available()) {  // If data is received
        x_data = Wire.read();
        y_data = Wire.read();
        button_data = Wire.read();
        
        // Print joystick data to the serial monitor
        sprintf(data, "Joystick x:%d y:%d button:%d\n", x_data, y_data, button_data);
        Serial.print(data);
    }

    // ********** Handle seesaw gamepad **********
    // Read analog values for x and y from seesaw
    int x = 1023 - ss.analogRead(14);  // Reverse x value
    int y = 1023 - ss.analogRead(15);  // Reverse y value

    // Print x and y values if significant change is detected
    if ((abs(x - last_x) > 3) || (abs(y - last_y) > 3)) {
        Serial.print("Gamepad x: "); Serial.print(x); Serial.print(", y: "); Serial.println(y);
        last_x = x;
        last_y = y;
    }

    // Read button states
    uint32_t buttons = ss.digitalReadBulk(button_mask);

    if (!(buttons & (1UL << BUTTON_A))) {
        Serial.println("Button A pressed");
    }
    if (!(buttons & (1UL << BUTTON_B))) {
        Serial.println("Button B pressed");
    }
    if (!(buttons & (1UL << BUTTON_Y))) {
        Serial.println("Button Y pressed");
    }
    if (!(buttons & (1UL << BUTTON_X))) {
        Serial.println("Button X pressed");
    }
    if (!(buttons & (1UL << BUTTON_SELECT))) {
        Serial.println("Button SELECT pressed");
    }
    if (!(buttons & (1UL << BUTTON_START))) {
        Serial.println("Button START pressed");
    }

    delay(200);  // Optional delay to avoid flooding the serial output
}
