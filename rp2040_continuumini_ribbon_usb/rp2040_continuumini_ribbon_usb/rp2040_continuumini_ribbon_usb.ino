#include <SPI.h>
#include <Wire.h>
#include <stdlib.h>  // For random number generation


int userChannel = 1;  // 1-16
int userProgOffset = 0;
bool serialDebug = false;

#include <MIDI.h>

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

// Function to send random MIDI note
void sendRandomMidiNote() {
    byte randomNote = random(60, 72); // Generate a random note between C4 (60) and B4 (72)
    byte randomVelocity = 127; //random(50, 127); // Random velocity between 50 and 127

    // Send the note to the USB MIDI and UART MIDI
    hostMidi->sendNoteOn(randomNote, randomVelocity, userChannel);

    usbhMIDI.writeFlushAll();
    //MIDIuart.sendNoteOn(randomNote, randomVelocity, userChannel);
    if (serialDebug){
      Serial.printf("Sent random note on#%u, velocity=%u\r\n", randomNote, randomVelocity);
    }

    // After 250ms, turn the note off to prevent sustained notes
    delay(250);
    hostMidi->sendNoteOff(randomNote, 0, userChannel);
    //MIDIuart.sendNoteOff(randomNote, 0, userChannel);
    if (serialDebug){
      Serial.printf("Sent note off#%u\r\n", randomNote);
    }
    usbhMIDI.writeFlushAll();
}

static void midiPanic() {
    for (int i = 0; i < 128; i++) {
        //MIDIusb.sendNoteOff(i, 0, userChannel);
        //MIDIuart.sendNoteOff(i, 0, userChannel);
        if (serialDebug){
          Serial.printf("note %u off\r\n", i);
        }
        last_cc_cntrl = 0;  // dirty this
    }
}

static void onNoteOff(Channel channel, byte note, byte velocity) {
    //MIDIusb.sendNoteOff(note, velocity, userChannel);
    // MIDIuart.sendNoteOff(note, velocity, userChannel);
    if (serialDebug){
      Serial.printf("ch%u: Note off#%u v=%u\r\n", userChannel, note, velocity);
    }
    last_cc_cntrl = 0;
}

static void onNoteOn(Channel channel, byte note, byte velocity) {
    // MIDIusb.sendNoteOn(note, velocity, userChannel);
    // MIDIuart.sendNoteOn(note, velocity, userChannel);
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
    //MIDIusb.sendAfterTouch(note, amount, userChannel);
    // MIDIuart.sendAfterTouch(note, amount, userChannel);
}


static void onControlChange(Channel channel, byte controller, byte value)
{
    //MIDIusb.sendControlChange(controller, value, userChannel);
    // MIDIuart.sendControlChange(controller, value, userChannel);
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
    //MIDIusb.sendProgramChange(program + userProgOffset, userChannel);
    // MIDIuart.sendProgramChange(program + userProgOffset, userChannel);
    last_cc_cntrl = 0;  // dirty this
    
}

static void onAftertouch(Channel channel, byte value)
{
  if (serialDebug){
    Serial.printf("ch%u: AT=%u\r\n", userChannel, value);
  }
    //MIDIusb.sendAfterTouch(value, userChannel);
    // MIDIuart.sendAfterTouch(value, userChannel);
}

static void onPitchBend(Channel channel, int value)
{
  if (serialDebug){
    Serial.printf("ch%u: PB=%d\r\n", userChannel, value);
  }
    //MIDIusb.sendPitchBend(value, userChannel);
    // MIDIuart.sendPitchBend(value, userChannel);
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
    //MIDIusb.sendSongPosition(beats);
    // MIDIuart.sendSongPosition(beats);
}

static void onSongSelect(byte songnumber)
{
  if (serialDebug){
    Serial.printf("SongS#%u\r\n", songnumber);
  }
    //MIDIusb.sendSongSelect(songnumber);
    // MIDIuart.sendSongSelect(songnumber);
}

static void onTuneRequest()
{
  if (serialDebug){
    Serial.printf("Tune\r\n");
  }
    //MIDIusb.sendTuneRequest();
    // MIDIuart.sendTuneRequest();
}

static void onMidiClock()
{
  if (serialDebug){
    Serial.printf("Clock\r\n");
  }
    //MIDIusb.sendClock();
    // MIDIuart.sendClock();
}

static void onMidiStart()
{
  if (serialDebug){
    Serial.printf("Start\r\n");
  }
    //MIDIusb.sendStart();
    // MIDIuart.sendStart();
}

static void onMidiContinue()
{
  if (serialDebug){
    Serial.printf("Cont\r\n");
  }
    //MIDIusb.sendContinue();
    // MIDIuart.sendContinue();
}

static void onMidiStop()
{
  if (serialDebug){
    Serial.printf("Stop\r\n");
  }
    //MIDIusb.sendStop();
    //  MIDIuart.sendStop();
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
    if (serialDebug){
      Serial.begin(115200);
    }
    pinMode(LED_BUILTIN, OUTPUT);
    if (serialDebug){
      Serial.println("USB Host to MIDI Messenger\r\n");
    }
    core0_booting = false;
    while (core1_booting);
}

void loop() {
    // Handle any incoming data; triggers MIDI IN callbacks
    usbhMIDI.readAll();
    // Do other processing that might generate pending MIDI OUT data

    // Tell the USB Host to send as much pending MIDI OUT data as possible
    usbhMIDI.writeFlushAll();

    // Do other non-USB host processing
    blinkLED();

    // Check if 200ms has passed since the last note
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        // Save the last time a note was sent
        previousMillis = currentMillis;

        // Send a random MIDI note
        sendRandomMidiNote();
    }
}
