#include <SPI.h>
#include <Wire.h>
#include "Adafruit_seesaw.h"
#include <SPI.h>
#include <stdlib.h>  // For random number generation
#include <MIDI.h>
#include "pio_usb.h"
#include "EZ_USB_MIDI_HOST.h"
#include "Adafruit_USBH_Host.h"

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

Adafruit_seesaw ss;
char data[100];
int last_x = 0, last_y = 0;
static uint8_t x_data, y_data, button_data;

int userChannel = 1;  // 1-16
int userProgOffset = 0;
bool serialDebug = true;

#if defined(USE_TINYUSB_HOST) || !defined(USE_TINYUSB)
#error "Please use the Menu to select Tools->USB Stack: Adafruit TinyUSB"
#endif
#include "pio_usb.h"
#define HOST_PIN_DP 16  // Pin used as D+ for host, D- = D+ + 1
#include "EZ_USB_MIDI_HOST.h"

// USB Host object
Adafruit_USBH_Host USBHost;

USING_NAMESPACE_MIDI
USING_NAMESPACE_EZ_USB_MIDI_HOST

RPPICOMIDI_EZ_USB_MIDI_HOST_INSTANCE(usbhMIDI, MidiHostSettingsDefault)

Adafruit_USBD_MIDI usb_midi;                                  // USB MIDI object


static uint8_t midiDevAddr = 0;
static bool core0_booting = true;
static bool core1_booting = true;
midi::MidiInterface<EZ_USB_MIDI_HOST_Transport<MidiHostSettingsDefault>, MidiHostSettingsDefault> * hostMidi;

/* MIDI IN MESSAGE REPORTING */
static void onMidiError(int8_t errCode)
{
  if (serialDebug){
    Serial.printf("MIDI Errors: %s %s %s\r\n", (errCode & (1UL << ErrorParse)) ? "Parse":"",
        (errCode & (1UL << ErrorActiveSensingTimeout)) ? "Active Sensing Timeout" : "",
        (errCode & (1UL << WarningSplitSysEx)) ? "Split SysEx":"");
  }
}

// Time tracking for random MIDI note generation
unsigned long previousMillis = 0;  // Store last time a note was sent
const long interval = 500;         // Interval at which to send a note (500ms)

int last_cc_cntrl = 1;



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


// Function to send random MIDI note
void sendRandomMidiNote() {
    byte randomNote = random(60, 72); // Generate a random note between C4 (60) and B4 (72)
    byte randomVelocity = 127; //random(50, 127); // Random velocity between 50 and 127

    // Send the note to the USB MIDI and UART MIDI
    hostMidi->sendNoteOn(randomNote, randomVelocity, userChannel);

    usbhMIDI.writeFlushAll();
    if (serialDebug){
      Serial.printf("Sent random note on#%u, velocity=%u\r\n", randomNote, randomVelocity);
    }

    // After 250ms, turn the note off to prevent sustained notes
    delay(250);
    hostMidi->sendNoteOff(randomNote, 0, userChannel);
    if (serialDebug){
      Serial.printf("Sent note off#%u\r\n", randomNote);
    }
    usbhMIDI.writeFlushAll();
}

static void midiPanic() {
    for (int i = 0; i < 128; i++) {
        if (serialDebug){
          Serial.printf("note %u off\r\n", i);
        }
        last_cc_cntrl = 0;  // dirty this
    }
}

static void onNoteOff(Channel channel, byte note, byte velocity) {
    if (serialDebug){
      Serial.printf("ch%u: Note off#%u v=%u\r\n", userChannel, note, velocity);
    }
    last_cc_cntrl = 0;
}

static void onNoteOn(Channel channel, byte note, byte velocity) {
    if (serialDebug){
      Serial.printf("ch%u: Note on#%u v=%u\r\n", userChannel, note, velocity);
    }
    last_cc_cntrl = 0;
}

static void onPolyphonicAftertouch(Channel channel, byte note, byte amount)
{
  if (serialDebug){
    Serial.printf("ch%u: PAT#%u=%u\r\n", userChannel, note, amount);
  }
}


static void onControlChange(Channel channel, byte controller, byte value)
{
    if (serialDebug){
      Serial.printf("Ch %u CC#%u=%u\r\n", userChannel, controller, value);
    }
    if (last_cc_cntrl != controller){
      last_cc_cntrl = controller;
    }
    
}

static void onProgramChange(Channel channel, byte program)
{
  if (serialDebug){
    Serial.printf("ch%u: Prog=%u\r\n", userChannel, program);
  }
    last_cc_cntrl = 0;  // dirty this
    
}

static void onAftertouch(Channel channel, byte value)
{
  if (serialDebug){
    Serial.printf("ch%u: AT=%u\r\n", userChannel, value);
  }
}

static void onPitchBend(Channel channel, int value)
{
  if (serialDebug){
    Serial.printf("ch%u: PB=%d\r\n", userChannel, value);
  }
}

static void onSysEx(byte * array, unsigned size)
{
  if (serialDebug){
    Serial.printf("SysEx:\r\n");
    unsigned multipleOf8 = size/8;
    unsigned remOf8 = size % 8;
    for (unsigned idx=0; idx < multipleOf8; idx++) {
        for (unsigned jdx = 0; jdx < 8; jdx++) {
            Serial.printf("%02x ", *array++);
        }
        Serial.printf("\r\n");
    }
    for (unsigned idx = 0; idx < remOf8; idx++) {
        Serial.printf("%02x ", *array++);
    }
    Serial.printf("\r\n");
  }
}

static void onSMPTEqf(byte data)
{
  if (serialDebug){
    uint8_t type = (data >> 4) & 0xF;
    data &= 0xF;    
    static const char* fps[4] = {"24", "25", "30DF", "30ND"};
    switch (type) {
        case 0: Serial.printf("SMPTE FRM LS %u \r\n", data); break;
        case 1: Serial.printf("SMPTE FRM MS %u \r\n", data); break;
        case 2: Serial.printf("SMPTE SEC LS %u \r\n", data); break;
        case 3: Serial.printf("SMPTE SEC MS %u \r\n", data); break;
        case 4: Serial.printf("SMPTE MIN LS %u \r\n", data); break;
        case 5: Serial.printf("SMPTE MIN MS %u \r\n", data); break;
        case 6: Serial.printf("SMPTE HR LS %u \r\n", data); break;
        case 7:
            Serial.printf("SMPTE HR MS %u FPS:%s\r\n", data & 0x1, fps[(data >> 1) & 3]);
            break;
        default:
          Serial.printf("invalid SMPTE data byte %u\r\n", data);
          break;
    }
  }
}

static void onSongPosition(unsigned beats)
{
  if (serialDebug){
    Serial.printf("SongP=%u\r\n", beats);
  }
}

static void onSongSelect(byte songnumber)
{
  if (serialDebug){
    Serial.printf("SongS#%u\r\n", songnumber);
  }
}

static void onTuneRequest()
{
  if (serialDebug){
    Serial.printf("Tune\r\n");
  }
}

static void onMidiClock()
{
  if (serialDebug){
    Serial.printf("Clock\r\n");
  }
}

static void onMidiStart()
{
  if (serialDebug){
    Serial.printf("Start\r\n");
  }
}

static void onMidiContinue()
{
  if (serialDebug){
    Serial.printf("Cont\r\n");
  }
}

static void onMidiStop()
{
  if (serialDebug){
    Serial.printf("Stop\r\n");
  }
}

static void onActiveSense()
{
  if (serialDebug){
    Serial.printf("ASen\r\n");
  }
}

static void onSystemReset()
{
  if (serialDebug){
    Serial.printf("SysRst\r\n");
  }
}

static void onMidiTick()
{
  if (serialDebug){
    Serial.printf("Tick\r\n");
  }
}

static void onMidiInWriteFail(uint8_t devAddr, uint8_t cable, bool fifoOverflow)
{
  if (serialDebug){
    if (fifoOverflow)
        Serial.printf("Dev %u cable %u: MIDI IN FIFO overflow\r\n", devAddr, cable);
    else
        Serial.printf("Dev %u cable %u: MIDI IN FIFO error\r\n", devAddr, cable);
  }
}

static void registerMidiInCallbacks()
{
    midi::MidiInterface<EZ_USB_MIDI_HOST_Transport<MidiHostSettingsDefault>, MidiHostSettingsDefault> * intf = usbhMIDI.getInterfaceFromDeviceAndCable(midiDevAddr, 0);
    if (intf == nullptr)
        return;


    hostMidi = intf;
    intf->setHandleNoteOff(onNoteOff);                      // 0x80
    intf->setHandleNoteOn(onNoteOn);                        // 0x90
    intf->setHandleAfterTouchPoly(onPolyphonicAftertouch);  // 0xA0
    intf->setHandleControlChange(onControlChange);          // 0xB0
    intf->setHandleProgramChange(onProgramChange);          // 0xC0
    intf->setHandleAfterTouchChannel(onAftertouch);         // 0xD0
    intf->setHandlePitchBend(onPitchBend);                  // 0xE0
    intf->setHandleSystemExclusive(onSysEx);                // 0xF0, 0xF7
    intf->setHandleTimeCodeQuarterFrame(onSMPTEqf);         // 0xF1
    intf->setHandleSongPosition(onSongPosition);            // 0xF2
    intf->setHandleSongSelect(onSongSelect);                // 0xF3
    intf->setHandleTuneRequest(onTuneRequest);              // 0xF6
    intf->setHandleClock(onMidiClock);                      // 0xF8
    // 0xF9 as 10ms Tick is not MIDI 1.0 standard but implemented in the Arduino MIDI Library
    intf->setHandleTick(onMidiTick);                        // 0xF9
    intf->setHandleStart(onMidiStart);                      // 0xFA
    intf->setHandleContinue(onMidiContinue);                // 0xFB
    intf->setHandleStop(onMidiStop);                        // 0xFC
    intf->setHandleActiveSensing(onActiveSense);            // 0xFE
    intf->setHandleSystemReset(onSystemReset);              // 0xFF
    intf->setHandleError(onMidiError);

    auto dev = usbhMIDI.getDevFromDevAddr(midiDevAddr);
    if (dev == nullptr)
        return;
    dev->setOnMidiInWriteFail(onMidiInWriteFail);
}

// CONNECTION MANAGEMENT
static void onMIDIconnect(uint8_t devAddr, uint8_t nInCables, uint8_t nOutCables) {
  if (serialDebug){
    Serial.printf("MIDI device at address %u has %u IN cables and %u OUT cables\r\n", devAddr, nInCables, nOutCables);
  }
    midiDevAddr = devAddr;
    registerMidiInCallbacks();
}

static void onMIDIdisconnect(uint8_t devAddr) {
  if (serialDebug){
    Serial.printf("MIDI device at address %u unplugged\r\n", devAddr);
  }
    midiDevAddr = 0;
}

// MAIN LOOP FUNCTIONS
static void blinkLED(void) {
    const uint32_t intervalMs = 1000;
    static uint32_t startMs = 0;
    static bool ledState = false;
    if (millis() - startMs < intervalMs) {
        return;
    }
    startMs += intervalMs;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
}

// core1's setup
void setup1() {
    #if ARDUINO_ADAFRUIT_FEATHER_RP2040_USB_HOST
        pinMode(18, OUTPUT);  // Sets pin USB_HOST_5V_POWER to HIGH to enable USB power
        digitalWrite(18, HIGH);
    #endif

    if (serialDebug){
        Serial.println("Core1 setup to run TinyUSB host with pio-usb\r\n");
    }

    uint32_t cpu_hz = clock_get_hz(clk_sys);
    if (cpu_hz != 120000000UL && cpu_hz != 240000000UL) {
        delay(2000);  // wait for native usb
        if (serialDebug){
          Serial.printf("Error: CPU Clock = %lu, PIO USB requires CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
        }
        while (1) delay(1);
    }

    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    pio_cfg.pin_dp = HOST_PIN_DP;

    USBHost.configure_pio_usb(1, &pio_cfg);
    usbhMIDI.begin(&USBHost, 1, onMIDIconnect, onMIDIdisconnect);
    core1_booting = false;
    while (core0_booting);
}

// core1's loop
void loop1() {
    USBHost.task();
}

void setup() {
    TinyUSBDevice.setManufacturerDescriptor("ThereminHero");
    TinyUSBDevice.setProductDescriptor("MIDI Device");
    
    Serial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);
    if (serialDebug){
      Serial.println("USB Host to MIDI Messenger\r\n");
    }

    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);
    Wire.setSDA(SDA_PIN);
    Wire.setSCL(SCL_PIN);
    Wire.begin();

    // Initialize the Adafruit seesaw gamepad
    if (!ss.begin(0x50)) {
        Serial.println("ERROR! seesaw not found");
        while(1) delay(1);
    }

    // Configure button inputs on the seesaw
    ss.pinModeBulk(button_mask, INPUT_PULLUP);
    ss.setGPIOInterrupts(button_mask, 1);

    



    for (int i = 0; i < N_STR; i++) {
      pinMode(S_pins[i], INPUT);
    }

    pinMode(M0, INPUT);
    pinMode(M1, INPUT);
    mod_init[0] = analogRead(M0);
    mod_init[1] = analogRead(M1);
    s_init[1] = analogRead(S1);
    s_init[0] = analogRead(S0);

    core0_booting = false;
    while (core1_booting);
}

void loop() {
    // Handle any incoming data; triggers MIDI IN callbacks
    usbhMIDI.readAll();
    // Do other processing that might generate pending MIDI OUT data

    readControls();
    determineFrets();
    legatoTest();
    pickNotes();
    cleanUp();


    // Tell the USB Host to send as much pending MIDI OUT data as possible
    usbhMIDI.writeFlushAll();




    // Do other non-USB host processing
    delay(10);
    blinkLED();
    


    // Handle Joystick I2C communication
    Wire.requestFrom(JOY_ADDR, 3);
    if (Wire.available()) {
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
    
    

    // Check if 200ms has passed since the last note
    // unsigned long currentMillis = millis();
    // if (currentMillis - previousMillis >= interval) {
    //     // Save the last time a note was sent
    //     previousMillis = currentMillis;

    //     // Send a random MIDI note
    //     sendRandomMidiNote();
    // }
}





// Original ribbon controls

void pickNotes() {
  for (int i = 0; i < N_STR; i++) {
    if (S_active[i]) {
      //noteOff(0x80 + channel, S_active[i]);
    }
    if (fretTouched[i] == 1) {
      //noteOff(0x80 + channel, S_active[i]);
      continue;
    } else {
      S_active[i] = fretTouched[i] + offsets[i];
      //noteOn(0x90 + channel, S_active[i], 100);
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
      //noteOff(0x80 + channel, S_active[i]);
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
    hostMidi->sendNoteOn(pitch, velocity, userChannel);
    usbhMIDI.writeFlushAll();
    Serial.printf("Note On\r\n");
   
}

void noteOff(int cmd, int pitch) {
    hostMidi->sendNoteOff(pitch, 127, userChannel);
    usbhMIDI.writeFlushAll();
    Serial.printf("Note Off\r\n");
}

void controllerChange(int controller, int value) {
  // if (!debugMode) {
  //   int ch = 176 + channel;
  //   Serial.write(byte(ch));
  //   Serial.write(byte(controller));
  //   Serial.write(byte(value));
  //   Serial.flush();
  // }
}

void sendMidiMessage(byte param1, byte param2, byte channel) {
  // Serial.write(channel & 0x0F);
  // Serial.write(0b11100000);
  // Serial.write(param1 & 0x7F);
  // Serial.write(param2 & 0x7F);
  // Serial.flush();
}

void sendPitchBendMidiMessage(int bend) {
  sendMidiMessage(bend & 0x7F, (bend >> 7) & 0x7F, 0x00);
}



