// JoystickWirelessTransmit.ino
// Revised: 12/08/2019
// Device: Arduino Nano

// Library Imports
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

const byte PIN_CE = 7;
const byte PIN_CSN = 8;
const byte PIN_POT_VERT = A0;
const byte PIN_POT_HORZ = A7;
const byte PIN_BTN_DOWN = 4;
const byte PIN_BTN_UP = 3;
const byte PIN_BTN_EXT = 9;
const byte PIN_SW_SCALE = A5;

RF24 radio(PIN_CE, PIN_CSN);
uint8_t data[3];
const byte CH = 80;
const uint64_t PIPE = 0xE8E8F0F0E1LL;

uint16_t vert_value;
uint16_t horz_value;
int8_t enc_ticks[2];

int8_t header = 0b00;
const byte HDR_PLOW_DOWN = 0;
const byte HDR_PLOW_UP = 1;

const int T_SAMP = 125;

void setup() {
  pinMode(PIN_POT_VERT, INPUT);
  pinMode(PIN_POT_HORZ, INPUT);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
  pinMode(PIN_BTN_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_EXT, INPUT_PULLUP);
  pinMode(PIN_SW_SCALE, INPUT_PULLUP);

  radio.begin();
  radio.setChannel(CH);
  radio.openWritingPipe(PIPE);

  Serial.begin(19200);
}

void loop() {
  if(digitalRead(PIN_BTN_DOWN)) {
    bitClear(header, HDR_PLOW_DOWN);
  } else {
    bitSet(header, HDR_PLOW_DOWN);
  }
  if(digitalRead(PIN_BTN_UP)) {
    bitClear(header, HDR_PLOW_UP);
  } else {
    bitSet(header, HDR_PLOW_UP);
  }
  
  adc();
  adcToEnc();

  data[0] = header;
  data[1] = enc_ticks[0];
  data[2] = enc_ticks[1];
  wirelessSend();
  
  delay(T_SAMP);
}

void adc() {
  vert_value = analogRead(PIN_POT_VERT);
  horz_value = 1023 - analogRead(PIN_POT_HORZ);
}

void adcToEnc() {
  if(!digitalRead(PIN_BTN_EXT)) {
    if(horz_value < 462) {
      enc_ticks[0] = int8_t(((461 - horz_value) * 35.0) / 461.0);
      enc_ticks[1] = -1 * enc_ticks[0];
    } else if(horz_value > 562) {
      enc_ticks[1] = int8_t(((horz_value - 563) * 35.0) / 461.0);
      enc_ticks[0] = -1 * enc_ticks[1];
    } else {
      enc_ticks[0] = 0;
      enc_ticks[1] = 0;
    }
  } else {
    if(vert_value < 462) {
      if(horz_value < 462) {
        enc_ticks[0] = -1 * int8_t(((461 - vert_value) * 35.0) / 461.0);
        enc_ticks[1] = int8_t((int16_t(enc_ticks[1]) * int16_t(horz_value)) / 461.0);
      } else if(horz_value > 562) {
        enc_ticks[1] = -1 * int8_t(((421 - vert_value) * 35.0) / 461.0);
        enc_ticks[0] = int8_t((int16_t(enc_ticks[0]) * int16_t(461 - (horz_value - 563))) / 461.0);
      } else {
        enc_ticks[0] = -1 * int8_t(((461 - vert_value) * 35.0) / 461.0);
        enc_ticks[1] = enc_ticks[0];
      }
    } else if(vert_value > 562) {
      if(horz_value < 462) {
        enc_ticks[0] = int8_t(((vert_value - 563) * 35.0) / 461.0);
        enc_ticks[1] = int8_t((enc_ticks[1] * horz_value) / 461.0);
      } else if(horz_value > 562) {
        enc_ticks[1] = int8_t(((vert_value - 563) * 35.0) / 461.0);
        enc_ticks[0] = int8_t((enc_ticks[1] * (461 - (horz_value - 563))) / 461.0);
      } else {
        enc_ticks[0] = int8_t(((vert_value - 563) * 35.0) / 461.0);
        enc_ticks[1] = enc_ticks[0];
      }
    } else {
      enc_ticks[0] = 0;
      enc_ticks[1] = 0;
    }
  }
  if(!digitalRead(PIN_SW_SCALE)) {
    enc_ticks[0] /= 2;
    enc_ticks[1] /= 2;
  }
}

void wirelessSend() {
  radio.write(data, sizeof(data));
}
