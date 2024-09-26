/*
  Arduino Ribbon Synth MIDI controller
  ------------------------------------
  ©2015 Dean Miller
  Modified by hyz, greigs
*/

#include <DueFlashStorage.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <QuickStats.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#define PIN_LED   13
#define NEO_PIN   21
#define N_PIXELS  29      //# of LEDS
#define TRANSPOSE_UP 4    //D4
#define TRANSPOSE_DOWN 2  //D2
#define VOLUME    A8      //Master volume
#define MODAL     A1      //Switch between the 7 musical modes
#define S0        A2      //String 0
#define M0        A3      //Modulation 0
#define S1        A4      //String 1
#define M1        A5      //Modulation 1

/*Here for legacy*/
#define T0        A0
#define T1        A0
#define T2        A0
#define T3        A0

#define JSX       A7
#define JSY       A6

#define THRESH    600
#define N_STR     2
#define N_FRET    2
#define S_PAD     3
#define T_PAD     300

#define MOD_THRESHOLD 30  //Modulation is not send under this value

//---Midi CC----
#define VOLUME_CC 7
#define MOD_CC 1
#define MIDI_CHANNEL 0
#define VOLCA_VOLUME_CC 11
#define VOLCA_MOD_CC 46
#define VOLCA_MIDI_CHANNEL 10
#define MUTE_CC 123

long noteDebounceTime = 0;
int noteDebounceDelay = 25;

long lastDebounceTime = 0;
int debounceDelay = 200;

long ledDebounceTime = 0;
int ledDebounceDelay = 20;

uint16_t fretDefs[N_STR][N_FRET];

int mod_final;
int vol;
int vol_buffer;
int modal;
int modal_buffer;
int buffer_mod[2];
int mod[2];
int mod_init[2]; //initial values for modulation
int s_init[2];   //intial values for string position
int pre_vol;     //previous volume
int pre_mod;     //previous modulation
bool volca = false;
int volume_cc = VOLUME_CC;
int mod_cc = MOD_CC;
int channel = MIDI_CHANNEL;
bool isPitchBend = false;
unsigned int pitchBendLight = 0;
bool dim = false;


int modal_array [6][7] =  {{0, 2, 4, 5, 7, 9, 11}, //ionian
  {0, 2, 3, 5, 7, 9, 10}, //dorian
  {0, 1, 3, 5, 7, 8, 10}, //phyrgian
  {0, 2, 4, 6, 7, 9, 11}, //lydian
  {0, 2, 4, 5, 7, 9, 10}, //mxyolydian
  {0, 2, 3, 5, 7, 8, 10}
};    //aeolian

uint16_t T_vals[N_STR];
bool T_active[] = {false, false, false, false}; //is it currently active
uint16_t T_hit[N_STR];                             //has it been hit on this loop
int T_pins[] = {T0, T1, T2, T3};

uint16_t S_vals[N_STR];                            //current sensor values
uint16_t S_old[N_STR];                             //old sensor values for comparison
int S_active[N_STR];                            //currently active notes
int S_pins[] = {S0, S1};
int fretTouched[N_STR];

bool modreset = true;

//E A D G
int offsets_default[] = {40, 45, 50, 55};

//B E A D
int offsets_transposed[] = {35, 40, 45, 50};

//default offsets
int offsets[] = {40, 45, 50, 55};


Adafruit_NeoPixel pixels = Adafruit_NeoPixel(N_PIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);
int led_number[2] = {0, 0};
int led_color;
int led;
int prev_led;

bool stickActive = false;
bool stickState = false;
bool btnState = false;
int stickZeroX;
int stickZeroY;

unsigned long last_read;

DueFlashStorage flashStorage;
uint16_t storageValidValue = 101;
int storageValidAddress = 301;

bool calibrateMode = false;
bool debugMode = false;

QuickStats stats;

void setup() {

  if (!storageIsValid()) {
    calibrateMode = true;
    debugMode = true; // keep this on until the arduino is reset
  }

  if (calibrateMode) {
    Serial.begin(115200);
  }
  else {
    //read fret definitions from Flash storage
    for (int i = 0; i < N_STR; i++) {
      for (int j = 0; j < N_FRET; j++) {
        fretDefs[i][j] = FlashReadInt(j * sizeof(uint16_t) + (N_FRET * i * sizeof(uint16_t)));
      }
    }
    Serial.begin(31250);
  }



  for (int i = 0; i < N_STR; i++) {
    pinMode(T_pins[i], INPUT);
    pinMode(S_pins[i], INPUT);
  }

  pinMode(JSX, INPUT);
  pinMode(JSY, INPUT);

  pinMode(PIN_LED, OUTPUT);

  pixels.begin();
  pixels.setBrightness(50);
  pixels.show();
  //calibrate joystick
  stickZeroX = analogRead(JSX);
  stickZeroY = analogRead(JSY);

  // Should be done only once
  if (calibrateMode) {
    calibrate();

    // mark the calibration as valid, until the arduino is re-flashed.
    setStorageAsValid();

    calibrateMode = false;
  }

  pinMode(TRANSPOSE_UP, INPUT_PULLUP);
  pinMode(TRANSPOSE_DOWN, INPUT_PULLUP);
  pinMode(A0, INPUT);
  pinMode(3, INPUT_PULLUP);
  pinMode(VOLUME, INPUT);
  pinMode(M0, INPUT);
  pinMode(M1, INPUT);
  mod_init[0] = analogRead(M0);
  mod_init[1] = analogRead(M1);
  s_init[1] = analogRead(S1);
  s_init[0] = analogRead(S0);
}

void loop() {
  //readJoystick();
  readButtons();
  readModulationAndVol();
  readControls();
  determineFrets();
  legatoTest();
  pickNotes();
  cleanUp();
  delay(1);
}


bool storageIsValid() {
  return FlashReadInt(storageValidAddress) == storageValidValue;
}

void setStorageAsValid() {
  FlashWriteInt(storageValidAddress, storageValidValue);
}

void FlashWriteInt(int address, uint16_t value) {
  //One = Most significant -> Two = Least significant byte
  byte two = (value & 0xFF);
  byte one = ((value >> 8) & 0xFF);

  //Write the 4 bytes into the eeprom memory.
  flashStorage.write(address, two);
  flashStorage.write(address + 1, one);
}

uint16_t FlashReadInt(int address) {
  //Read the 2 bytes from the eeprom memory.
  long two = flashStorage.read(address);
  long one = flashStorage.read(address + 1);

  //Return the recomposed uint16_t by using bitshift.
  return ((two << 0) & 0xFF) + ((one << 8) & 0xFFFF);
}

void readModulationAndVol() {
  buffer_mod[0] = analogRead(M0);
  buffer_mod[1] = analogRead(M1);
  vol_buffer = analogRead(VOLUME);
  modal_buffer = analogRead(MODAL);
  modal_buffer = map(modal_buffer, 0, 700, 0, 7);
  mod[1] = map(buffer_mod[1], mod_init[1], mod_init[1] + 400, 0, 127);
  mod[0] = map(buffer_mod[0], 500, 500 + 300, 0, 127);
  mod_final = max(mod[0], mod[1]);
  vol = map(vol_buffer, 0, 300, 0, 127);
  if (abs(modal_buffer != modal)) {
    if (modal_buffer > 7) {
      modal = 7;
      modal_buffer = 7;
    }
    else
      modal = modal_buffer;
    onLED(modal + 1, 0, 255, 0);
    delay(500);
    onLED(N_PIXELS, 0, 0, 0);
    //Serial.println(modal);
  }

  if (abs(vol - pre_vol) > 1 && vol <= 127) {
    //Serial.println("vol");
    if (vol >= 127)
      vol = 127;
    if (vol <= 1)
      vol = 0;
    controllerChange(volume_cc, vol);
    pre_vol = vol;
  }
  if (abs(mod_final - pre_mod) > 5) {

    if (mod_final < MOD_THRESHOLD )
      controllerChange(mod_cc, 0);
    else if ( mod_final <= 127 )
      controllerChange(mod_cc, mod_final);
    //Serial.println("mod");
    pre_mod = mod_final;
  }
}

void readButtons() {
  int up = digitalRead(TRANSPOSE_UP);
  int down = digitalRead(TRANSPOSE_DOWN);
  int clear_notes = digitalRead(3);
  //Serial.println(up);
  if ((millis() - lastDebounceTime ) > debounceDelay) {
    //Serial.println("tranposeing");
    if (!down && !clear_notes)
      transpose(-2);
    else if (!up && !clear_notes)
      transpose(2);
    else if (!up && down)
      transpose(1);
    else if (!down && up)
      transpose(-1);
    else if (!up && !down) {
      if (!volca) {
        volume_cc = VOLCA_VOLUME_CC;
        mod_cc = VOLCA_MOD_CC;
        channel = VOLCA_MIDI_CHANNEL;
        volca = true;
        //Serial.println("volca");
      }
      else if (volca) {
        volume_cc = VOLUME_CC;
        mod_cc = MOD_CC;
        channel = MIDI_CHANNEL;
        volca = false;
        //Serial.println("!volca");
      }
    }

    //    else if (!clear_notes){
    //      controllerChange(MUTE_CC,0);
    //    }
    lastDebounceTime = millis();
  }
}


void pickNotes() {
  for (int i = 0; i < N_STR; i++) {
    if (T_hit[i]) {
      if (S_active[i]) {
        //turn off active note on this string

        if (debugMode) {
          Serial.print("picknoteoff: ");
          Serial.println(S_active[i]);
        }
        noteOff(0x80 + channel, S_active[i]);

      }
      if (fretTouched[i] == 1) {
        if (debugMode) {
          Serial.print("picknoteoff: ");
          Serial.println(S_active[i]);
        }
        noteOff(0x80 + channel, S_active[i]);
        continue;
      }
      else {
        S_active[i] = fretTouched[i] + offsets[i];

        if (debugMode) {
          Serial.print("picknoteon: ");
          Serial.println(S_active[i]);
        }
        noteOn(0x90 + channel, S_active[i], 100);

      }

    }
  }
}

void legatoTest() {
  for (int i = 0; i < N_STR; i++) {
    if (S_active[i]) {

	 if (i == 0)
	 {
		 
		 uint16_t s_val = S_vals[i];
		 
		// midi pitch bend has 0 - 16383 with midpoint 8192
		// 16 bit unsigned int is 0 - 65535
 
		// map(value, fromLow, fromHigh, toLow, toHigh)

		// Parameters
		// value: the number to map.
		// fromLow: the lower bound of the value’s current range.
		// fromHigh: the upper bound of the value’s current range.
		// toLow: the lower bound of the value’s target range.
		// toHigh: the upper bound of the value’s target range.
		 sendPitchBendMidiMessage(map(s_val,0,65535,0,16383));
	 }

      int note = fretTouched[i] + offsets[i];
      if (note != S_active[i] && fretTouched[i] == -1) {

        if (debugMode) {
          Serial.print("legatonote_off: ");
          Serial.println(S_active[i]);
        }

        noteOff(0x80 + channel, S_active[i]);

        S_active[i] = note;
        //clrLED();
        continue;

      }

      if (note != S_active[i] && (fretTouched[i] || T_active[i])) {
        //Serial.println("legatonote");
        int volume = mod_final * 2;
        if (volume > 127) {
          volume = 127;
        }

        if (debugMode) {
          Serial.print("legatonote_on: ");
          Serial.println(note);
          Serial.print("legatonote_off: ");
          Serial.println(S_active[i]);
        }

        // switch these around (temporary)
		noteOff(0x80 + channel, S_active[i]);
        noteOn(0x90 + channel, note, 127);
        

        S_active[i] = note;
      }
    }
  }

  led = max(led_number[0], led_number[1]);
  //Serial.println(led_number[0]);
  //Serial.println(led_number[1]);
  if (led == -1) {
    led = 0;
    onLED(N_PIXELS, 0, 0, 0);
  }

  else {
    led = led + 1;
    led = map(led, 0, N_FRET - 1, 0, N_PIXELS);
    led_color = map(led, 0, 30, 0, 255);
    //Serial.println(led_number);
    if ((millis() - ledDebounceTime ) > ledDebounceDelay) {
      for (int i = 0; i < led; i++) {
        //pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
        pixels.setPixelColor(i, Wheel(led_color));
      }
      if (prev_led > led)
        for (int i = led; i < N_PIXELS; i++) {
          //pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
          pixels.setPixelColor(i, (pixels.Color(0, 0, 0)));
        }
      pixels.show();
      ledDebounceTime = millis();
      prev_led = led ;
    }
  }
}

void cleanUp() {
  for (int i = 0; i < N_STR; i++) {
    if (S_active[i] && !fretTouched[i] && !T_active[i]) {
      if (debugMode) {
        Serial.print("cleanUp_noteoff: ");
        Serial.println(S_active[i]);
      }
      noteOff(0x80 + channel, S_active[i]);
      S_active[i] = 0;
    }
  }
}

void readControls() {
  //read the strings and the triggers
  for (int i = 0; i < N_STR; i++) {
    T_hit[i] = checkTriggered(i);
    //Serial.println(T_hit[i]);
    //if(i == 1 && abs(buffer_mod[i] - mod_init[i] > 1)){
    float temp[3];
    analogReadResolution(16);
    for (int k = 0 ; k < 3; k++) {
      temp[k] = analogRead(S_pins[i]);
      delay(3);
    }
    analogReadResolution(10);
    S_vals[i] = stats.minimum(temp, 3);
  }
}

void determineFrets () {
  //---------Get Fret Numbers------
  for (int i = 0; i < N_STR; i++) {

    uint16_t s_val = S_vals[i];

    //check for open strings
    if (s_val == 0 ) {
      S_old[i] = s_val;
      fretTouched[i] = -1;
      led_number[i] = fretTouched[i];
    }

    else {
      //loop through the array of fret definitions
      for (int j = 1; j < N_FRET; j++) {
        int k = j - 1;
        if (s_val >= fretDefs[i][j] &&
            s_val < fretDefs[i][k] &&
            abs((int)s_val - (int)S_old[i]) > S_PAD) {

          S_old[i] = s_val;
          fretTouched[i] = j - 1;
          led_number[i] = fretTouched[i];
          if (modal < 7) {
            // not chromatic mode
            if (i == 0)
              fretTouched[i] = modal_array[modal][fretTouched[i] % 7] + (fretTouched[i] / 7) * 12;
            else if (i == 1)
            {
              if (modal == 3)
                fretTouched[i] = (modal_array[(modal + 3) % 6][fretTouched[i] % 7] + (fretTouched[i] / 7) * 12) + 2; //fix for locrian
              else if (modal > 3) {
                fretTouched[i] = (modal_array[(modal + 2) % 6][fretTouched[i] % 7] + (fretTouched[i] / 7) * 12);
              }
              else {
                fretTouched[i] = modal_array[(modal + 3) % 6][fretTouched[i] % 7] + (fretTouched[i] / 7) * 12;
              }
            }

          }
          //        Serial.println("fret");
          //        Serial.println(i);
          //        Serial.println(fretTouched[i]);
          //        Serial.println("");
        }
      }
    }
  }

}
void calibrate() {
  Serial.println("calibrating...");
  for (int i = 0; i < N_STR; i++) {
    //Flash the LED too indicate calibration
    onLED(10, 250, 0, 0);
    delay(100);
    clrLED();
    onLED(10, 250, 0, 0);
    delay(100);
    clrLED();
    onLED(10, 250, 0, 0);
    delay(100);
    clrLED();

    uint16_t val;

    //loop through the array of fret definitions
    for (int j = N_FRET - 1; j >= 0; j--) {
      int response = false;
      //wait for response
      Serial.println("waiting");
      while (!response) {
        if (checkTriggered(i)) {
          analogReadResolution(16);
          val = (analogRead(S_pins[i]));
          analogReadResolution(10);
          response = true;
          //write to memory
          clrLED();
          int addr = j * sizeof(uint16_t) + (N_FRET * i * sizeof(uint16_t));
          Serial.print("Writing ");
          Serial.print(val);
          Serial.print(" to address: ");
          Serial.println(addr);
          FlashWriteInt(addr, val);
        }
        delay(10);
      }
      delay(100);
      onLED(10, 250, 0, 0);
    }

    for (int j = 0; j < N_FRET; j++) {
      uint16_t v = FlashReadInt(j * sizeof(uint16_t) + (N_FRET * i * sizeof(uint16_t)));
      Serial.print("Read ");
      Serial.print(v);
      Serial.print(" from address: ");
      Serial.println(j * sizeof(uint16_t) + (N_FRET * i * sizeof(uint16_t)));
      fretDefs[i][j] = v;
    }
  }
  clrLED();
}


void onLED(int led, int red, int green, int blue) {
  for (int i = 0; i < led; i++) {
    //pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(red, green, blue));
  }
  pixels.show();
}

void clrLED() {
  for (int i = 0; i < N_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 0)); // turn off
  }
  pixels.show();
}

//check if a trigger has been hit. Return 0 if not, the value of the trigger if it has
uint16_t checkTriggered(int i) {

  uint16_t v = analogRead(T_pins[i]);
  //Serial.println(v);
  T_vals[i] = v;
  uint16_t ret = 0;
  if (!T_active[i] && v > THRESH) {
    T_active[i] = true;
    ret = v;
    if (calibrateMode) {
      Serial.println("triggered");
    }
  }
  else if (T_active[i] && v < THRESH - T_PAD) {
    T_active[i] = false;
    if (calibrateMode) {
      Serial.println("un-triggered");
    }
  }
  //Serial.println(ret);
  return ret;
}

void transpose(int dir) {
  switch (dir) {
    case 1:
      for (int i = 0; i < N_STR; i++) {
        offsets[i] = offsets[i] + 1;
        //Serial.println(offsets[0]);
      }
      break;

    case -1:
      for (int i = 0; i < N_STR; i++) {
        offsets[i] = offsets[i] - 1;
        //Serial.println(offsets[0]);
      }
      break;
    case 2:
      for (int i = 0; i < N_STR; i++) {
        offsets[i] = offsets[i] + 12;
        //Serial.println(offsets[0]);
      }
      break;
    case -2:
      for (int i = 0; i < N_STR; i++) {
        offsets[i] = offsets[i] - 12;
        //Serial.println(offsets[0]);
      }
      break;
  }
}

void readJoystick() {
  unsigned int joyx = analogRead(JSX);
  pitchBendLight = (abs(joyx - 512) / 10);
  if (abs(joyx - 512) > 15) {
    isPitchBend = true;
    PitchWheelChange(map(joyx, 0, 1023, -8192, 8180));
  }
  else if (isPitchBend) {
    PitchWheelChange(512);
    isPitchBend = false;
  }
}

//note-on message
void noteOn(int cmd, int pitch, int velocity) {
  if (!debugMode) {
    Serial.write(byte(cmd));
    // Serial.write(byte(pitch));
	Serial.write(byte(0x3C));
    Serial.write(byte(velocity));
    Serial.flush();
  }
  digitalWrite(PIN_LED, HIGH);
}
//note-off message
void noteOff(int cmd, int pitch) {

  if (!debugMode) {
    Serial.write(byte(cmd));
    Serial.write(byte(0x3C));
    Serial.write(byte(0));
    Serial.flush();
  }
  digitalWrite(PIN_LED, LOW);
}

//Sends controller change to the specified controller
void controllerChange(int controller, int value) {
  //Serial.println("cc");

  if (!debugMode) {
    int ch = 176 + channel;
    Serial.write(byte(ch));
    Serial.write(byte(controller));
    Serial.write(byte(value));
    Serial.flush();
  }
}



// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return pixels.Color(255 - WheelPos * 3 + pitchBendLight, 0, WheelPos * 3 + pitchBendLight);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3 + pitchBendLight , 255 - WheelPos * 3 + pitchBendLight);
  }
  else {
    WheelPos -= 170;
    return pixels.Color(WheelPos * 3 + pitchBendLight , 255 - WheelPos * 3 + pitchBendLight , 0 );
  }
  if (mod_final > 50) {
  }
}


void PitchWheelChange(int value) {

  if (!debugMode)
  {
    unsigned int change = 0x2000 + value;  //  0x2000 == No Change
    unsigned char low = change & 0x7F;  // Low 7 bits
    unsigned char high = (change >> 7) & 0x7F;  // High 7 bits

    Serial.write(0xE0);
    Serial.write(low);
    Serial.write(high);
    Serial.flush();
  }
}


// int bend = constrain(pitchval + 0x2000, 0, 16383);
// sendPitchBendMidiMessage(MIDIPitchBend, bend & 0x7F, (bend >> 7) & 0x7F, channel);



void sendMidiMessage(byte param1, byte param2, byte channel) {
	Serial.write(channel & 0x0F);
	Serial.write(B11100000);	
	Serial.write(param1 & 0x7F);
	Serial.write(param2 & 0x7F);
	Serial.flush();
}

// 0 - 16383 with midpoint 8192
void sendPitchBendMidiMessage(int bend){
	sendMidiMessage(bend & 0x7F, (bend >> 7) & 0x7F, 0x00);
}
