#include <Wire.h>
#include <QuickStats.h>


#define S0        A2      //String 0
#define M0        A3      //Modulation 0
#define S1        A0      //String 1
#define M1        A1      //Modulation 1

#define THRESH    600
#define N_STR     1
#define N_FRET    1
#define S_PAD     3
#define T_PAD     300

#define MOD_THRESHOLD 30  //Modulation is not sent under this value

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

int mod_final;
int vol;
int vol_buffer;
int modal;
int modal_buffer;
int buffer_mod[2];
int mod[2];
int mod_init[2]; //initial values for modulation
int s_init[2];   //initial values for string position
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
  {0, 1, 3, 5, 7, 8, 10}, //phrygian
  {0, 2, 4, 6, 7, 9, 11}, //lydian
  {0, 2, 4, 5, 7, 9, 10}, //mixolydian
  {0, 2, 3, 5, 7, 8, 10}
};    //aeolian

uint16_t T_vals[N_STR];
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

bool stickActive = false;
bool stickState = false;
bool btnState = false;
int stickZeroX;
int stickZeroY;

unsigned long last_read;

uint16_t storageValidValue = 101;
int storageValidAddress = 301;

bool calibrateMode = false;
bool debugMode = false;

QuickStats stats;

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < N_STR; i++) {
    pinMode(S_pins[i], INPUT);
  }

  pinMode(M0, INPUT);
  pinMode(M1, INPUT);
  mod_init[0] = analogRead(M0);
  mod_init[1] = analogRead(M1);
  s_init[1] = analogRead(S1);
  s_init[0] = analogRead(S0);
}

void loop() {
  // readModulationAndVol();
  readControls();
  determineFrets();
  legatoTest();
  pickNotes();
  cleanUp();
  delay(1);
}

void pickNotes() {
  for (int i = 0; i < N_STR; i++) {
    if (S_active[i]) {
      noteOff(0x80 + channel, S_active[i]);
    }
    if (fretTouched[i] == 1) {
      noteOff(0x80 + channel, S_active[i]);
      continue;
    } else {
      S_active[i] = fretTouched[i] + offsets[i];
      noteOn(0x90 + channel, S_active[i], 100);
    }
  }
}

void legatoTest() {
  for (int i = 0; i < N_STR; i++) {
    if (S_active[i]) {
      if (i == 0 && fretTouched[0] > 1) {
        uint16_t s_val = S_vals[i];
        sendPitchBendMidiMessage(map(s_val, 0, 65535, 0, 16383));
      }
      int note = fretTouched[i] + offsets[i];
      if (note != S_active[i] && fretTouched[i] == -1) {
        noteOff(0x80 + channel, S_active[i]);
        S_active[i] = note;
        continue;
      }
      if (note != S_active[i] && (fretTouched[i])) {
        int volume = mod_final * 2;
        if (volume > 127) {
          volume = 127;
        }
        noteOff(0x80 + channel, S_active[i]);
        noteOn(0x90 + channel, note, 127);
        S_active[i] = note;
      }
    }
  }
}

void cleanUp() {
  for (int i = 0; i < N_STR; i++) {
    if (S_active[i] && !fretTouched[i]) {
      noteOff(0x80 + channel, S_active[i]);
      S_active[i] = 0;
    }
  }
}

void readControls() {
  for (int i = 0; i < N_STR; i++) {
    float temp[3];
    analogReadResolution(16);
    for (int k = 0; k < 3; k++) {
      temp[k] = analogRead(S_pins[i]);
      delay(3);
    }
    analogReadResolution(10);
    S_vals[i] = stats.minimum(temp, 3);
  }
}

void determineFrets() {
  for (int i = 0; i < N_STR; i++) {
    uint16_t s_val = S_vals[i];

    if (s_val < 1000) {
      S_old[i] = s_val;
      fretTouched[i] = -1;
    } else {
      if (s_val >= 1000 && s_val < 62000 && abs((int)s_val - (int)S_old[i]) > S_PAD) {
        S_old[i] = s_val;
        fretTouched[i] = 20;
      }
    }
  }
}

void readModulationAndVol() {
  buffer_mod[0] = analogRead(M0);
  buffer_mod[1] = analogRead(M1);
  vol_buffer = 300;
  modal_buffer = 0;
  modal_buffer = map(modal_buffer, 0, 700, 0, 7);
  mod[1] = map(buffer_mod[1], mod_init[1], mod_init[1] + 400, 0, 127);
  mod[0] = map(buffer_mod[0], 500, 500 + 300, 0, 127);
  mod_final = max(mod[0], mod[1]);
  vol = map(vol_buffer, 0, 300, 0, 127);
  if (abs(modal_buffer != modal)) {
    if (modal_buffer > 7) {
      modal = 7;
      modal_buffer = 7;
    } else {
      modal = modal_buffer;
    }
    delay(500);
  }

  if (abs(vol - pre_vol) > 1 && vol <= 127) {
    if (vol >= 127) vol = 127;
    if (vol <= 1) vol = 0;
    controllerChange(volume_cc, vol);
    pre_vol = vol;
  }
  if (abs(mod_final - pre_mod) > 5) {
    if (mod_final < MOD_THRESHOLD) {
      controllerChange(mod_cc, 0);
    } else if (mod_final <= 127) {
      controllerChange(mod_cc, mod_final);
    }
    pre_mod = mod_final;
  }
}

void transpose(int dir) {
  switch (dir) {
    case 1:
      for (int i = 0; i < N_STR; i++) {
        offsets[i] = offsets[i] + 1;
      }
      break;
    case -1:
      for (int i = 0; i < N_STR; i++) {
        offsets[i] = offsets[i] - 1;
      }
      break;
    case 2:
      for (int i = 0; i < N_STR; i++) {
        offsets[i] = offsets[i] + 12;
      }
      break;
    case -2:
      for (int i = 0; i < N_STR; i++) {
        offsets[i] = offsets[i] - 12;
      }
      break;
  }
}

void noteOn(int cmd, int pitch, int velocity) {
  if (!debugMode) {
    Serial.write(byte(cmd));
    Serial.write(byte(0x3C));
    Serial.write(byte(velocity));
    Serial.flush();
  }
  digitalWrite(PIN_LED, HIGH);
}

void noteOff(int cmd, int pitch) {
  if (!debugMode) {
    Serial.write(byte(cmd));
    Serial.write(byte(0x3C));
    Serial.write(byte(0));
    Serial.flush();
  }
  digitalWrite(PIN_LED, LOW);
}

void controllerChange(int controller, int value) {
  if (!debugMode) {
    int ch = 176 + channel;
    Serial.write(byte(ch));
    Serial.write(byte(controller));
    Serial.write(byte(value));
    Serial.flush();
  }
}

void sendMidiMessage(byte param1, byte param2, byte channel) {
  Serial.write(channel & 0x0F);
  Serial.write(0b11100000);
  Serial.write(param1 & 0x7F);
  Serial.write(param2 & 0x7F);
  Serial.flush();
}

void sendPitchBendMidiMessage(int bend) {
  sendMidiMessage(bend & 0x7F, (bend >> 7) & 0x7F, 0x00);
}
