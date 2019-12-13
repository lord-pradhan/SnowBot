// MotorControlWireless.ino
// Revised: 12/08/2019
// Device: Arduino MEGA 2560

// Library Imports
#include <TimerOne.h>
#include <TimerThree.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

// Pin Definitions
const byte PINS_ENC[] = {2, 3, 18, 19};   // motor optical encoder inputs [1, 2, 3, 4]
const byte PINS_PWM[] = {5, 6, 7, 8};     // motor driver pwm outputs [1, 2, 3, 4]
const byte PINS_DIR[] = {22, 23, 24, 25}; // motor driver direction outputs [1, 2, 3, 4]
const byte PIN_WIFI_TX = 16;              // ESP8266 transmit
const byte PIN_WIFI_RX = 17;              // ESP8266 receive
const byte PIN_RELAY_UP = 26;             // linear actuator relay expansion output 1
const byte PIN_RELAY_DOWN = 27;           // linear actuator relay contraction output 2
const byte PIN_LIMIT_UP = 28;             // linear actuator expansion limit switch input
const byte PIN_LIMIT_DOWN = A0;//29;           // linear actuator contraction limit switch input
const byte PIN_BUILTIN_LED = 13;

// PID Gains
const float KP[] = {10.0, 10.0, 10.0, 10.0}; // proportional gains [1, 2, 3, 4]
const float KI[] = {2.0, 2.0, 2.0, 2.0}; // integral gains [1, 2, 3, 4]
const float KD[] = {-2.5, -2.5, -2.5, -2.5}; // derivative gains [1, 2, 3, 4]

// Setpoints
volatile float setpoints[] = {0.0, 0.0, 0.0, 0.0};  // motor speed setpoints [1, 2, 3, 4] (ticks)

// PID Calculation Storage Variables
bool is_negative[] = {false, false, false, false};
float err_sum[] = {0.0, 0.0, 0.0, 0.0};             // error accumulation for integral control
float err_prev[] = {0.0, 0.0, 0.0, 0.0};            // error from previous time step for derivative control

// PID Control Efforts
int16_t u[] = {0, 0, 0, 0};
const int SLEW_LIMIT_UPPER = 128;
const int SLEW_LIMIT_LOWER = -1 * SLEW_LIMIT_UPPER;

// PID Integral Windup Clamping Bounds
const int ERR_SUM_MAX = 100;  // error accumulation upper bound
const int ERR_SUM_MIN = -100; // error accumulation lower bound

// Sampling Period
const long T_SAMP = 125000L;  // PID update interval (us)

// Encoder Counters
volatile unsigned short enc_count[] = {0, 0, 0, 0};

// Linear Actuator Timeout Fault
const unsigned int T_FAULT = 10000000;  // linear actuator fault timeout period (us)
bool timeout = false;

// Action Update Flags
volatile byte flags = 0x00;     // flag container
const byte FLAG_TIMER1 = 0;     // flag for Timer 1 expiration
const byte FLAG_PLOW_UP = 1;    // flag for plow raising command
const byte FLAG_PLOW_DOWN = 2;  // flag for plow lowering command

// Serial Baud Rate
const long BAUD = 19200L;

// Serial Constants
const byte LF = 0x0a;
const byte CR = 0x0d;

// Serial Buffer
volatile char serial_data[255];
volatile byte serial_count = 0;

// LED Toggle State;
bool led_on = false;

// Startup Delay Buffer
const short T_STARTUP_BUFF = 4000;  // startup delay (us)

// Radio Pins
const byte PIN_CE = 11;
const byte PIN_CSN = 12;

// Radio Routing
RF24 radio(PIN_CE, PIN_CSN);
int8_t data[3];
const byte CH = 80;
const uint64_t PIPE = 0xE8E8F0F0E1LL;
bool data_received = false;

void setup() {
  // Set up motor pin directions.
  for(byte i = 0; i < 4; i++) {
    pinMode(PINS_ENC[i], INPUT);  // set encoder pin 'i' as input
    pinMode(PINS_PWM[i], OUTPUT); // set pwm pin 'i' as output
    pinMode(PINS_DIR[i], OUTPUT); // set direction pin 'i' as output
  }

  // Set up serial pin directions.
  pinMode(PIN_WIFI_TX, OUTPUT); // set TX2 pin as output
  pinMode(PIN_WIFI_RX, INPUT);  // set RX2 pin as input
  
  // Set up linear actuator pin directions.
  pinMode(PIN_RELAY_UP, OUTPUT);    // set plow raising relay as output
  pinMode(PIN_RELAY_DOWN, OUTPUT);  // set plow lowering relay as output
  pinMode(PIN_LIMIT_UP, INPUT);     // set plow expansion limit switch as input
  pinMode(PIN_LIMIT_DOWN, INPUT_PULLUP);  // set plow contraction limit switch as input

  // Set up built-in LED pin direction.
  pinMode(PIN_BUILTIN_LED, OUTPUT);

  // Initialize outputs.
  for(int i = 0; i < 4; i++) {
    digitalWrite(PINS_PWM[i], LOW);
    digitalWrite(PINS_DIR[i], LOW);
  }
  digitalWrite(PIN_RELAY_UP, HIGH);
  digitalWrite(PIN_RELAY_DOWN, HIGH);
  digitalWrite(PIN_BUILTIN_LED, LOW);

  // Initialize radio.
  radio.begin();
  radio.setChannel(CH);
  radio.openReadingPipe(1, PIPE);
  radio.startListening();
  
  // Disable interrupts while setting up interrupts.
  noInterrupts();

  // Set up encoder interrupts.
  attachInterrupt(digitalPinToInterrupt(PINS_ENC[0]), encoder1ISR, CHANGE); // attach ISR to encoder 1
  attachInterrupt(digitalPinToInterrupt(PINS_ENC[1]), encoder2ISR, CHANGE); // attach ISR to encoder 2
  attachInterrupt(digitalPinToInterrupt(PINS_ENC[2]), encoder3ISR, CHANGE); // attach ISR to encoder 3
  attachInterrupt(digitalPinToInterrupt(PINS_ENC[3]), encoder4ISR, CHANGE); // attach ISR to encoder 4

  // Set up Timer 1 interrupt.
  Timer1.initialize(T_SAMP);
  Timer1.attachInterrupt(timer1ExpirationISR);

  // Delay startup.
  delay(T_STARTUP_BUFF);
  
  // Enable interrupts.
  interrupts();

  Serial.begin(BAUD);
}


void loop() {
  // Read radio.
  radioReceive();
  
  // Timer 1 expiration action.
  if(bitRead(flags, FLAG_TIMER1)) {
    bitClear(flags, FLAG_TIMER1);   // clear Timer 1 expiration flag
    led_on = !led_on;
    digitalWrite(PIN_BUILTIN_LED, led_on);
    updatePID();                    // update PID control
  }

  // Plow raise action.
  if(bitRead(flags, FLAG_PLOW_UP)) {
    bitClear(flags, FLAG_PLOW_UP);  // clear plow raising command flag
    forceStop();
    plowMove(true);                 // raise plow
  }

  // Plow lower action.
  if(bitRead(flags, FLAG_PLOW_DOWN)) {
    bitClear(flags, FLAG_PLOW_DOWN);  // clear plow lowering command flag
    forceStop();
    plowMove(false);                  // lower plow
  }
}


void radioReceive() {
  // Read radio if data is available.
  if(radio.available()) {
    data_received = true;
    while(radio.available()) {
      radio.read(&data, sizeof(data));
    }
//      delay(10);
  }

  // Update PID setpoints.
  if(data_received) {
    bitWrite(flags, FLAG_PLOW_DOWN, (data[0] & 0b01));
    bitWrite(flags, FLAG_PLOW_UP, (data[0] & 0b10) >> 1);
    if(!(bitRead(flags, FLAG_PLOW_DOWN) || bitRead(flags, FLAG_PLOW_UP))) {
      plowStop();
    }
    setpoints[0] = float(data[1]);
    setpoints[1] = float(data[1]);
    setpoints[2] = float(data[2]);
    setpoints[3] = float(data[2]);
  }
}


void updatePID() {
  // Declare local variables.
  long speed_curr[4];
  float err_curr[4];
  float err_diff[4];
  float up[4], ui[4], ud[4];
  int u_prev[4], slew[4];
  
  // Store encoder counts locally.
  speed_curr[0] = enc_count[0];
  speed_curr[1] = enc_count[0];//enc_count[1]; ENCODER BROKE
  speed_curr[2] = enc_count[2];
  speed_curr[3] = enc_count[3];

  // Reset encoder counts for next update cycle.
  resetEncoderCounts();

  for(int i = 0; i < 4; i++) {
    // Apply speed sign (positive or negative).
    if(is_negative[i]) {
      speed_curr[i] *= -1;
    }
    
    // Calculate current error.
    err_curr[i] = setpoints[i] - speed_curr[i];

    // Update error accumulation.
    err_sum[i] += err_curr[i];
    err_sum[i] = min(err_sum[i], ERR_SUM_MAX);  // Limit error accumulation upper bound.
    err_sum[i] = max(err_sum[i], ERR_SUM_MIN);  // Limit error accumulation lower bound.

    // Calculate error difference.
    err_diff[i] = err_curr[i] - err_prev[i];
    
    // Calculate PID control effort components.
    up[i] = KP[i] * err_curr[i];
    ui[i] = KI[i] * err_sum[i];
    ud[i] = KD[i] * err_diff[i];

    // Calculate net PID control effort.
    u_prev[i] = u[i];
    u[i] = int(up[i] + ui[i] + ud[i]);
    u[i] = min(u[i], 255);  // Saturate control effort at upper bound.
    u[i] = max(u[i], -255); // Saturate control effort at lower bound.

    // Limit slew rate.
    slew[i] = u[i] - u_prev[i];
    slew[i] = max(min(slew[i], SLEW_LIMIT_UPPER), SLEW_LIMIT_LOWER);
    u[i] = u_prev[i] + slew[i];

    // Store current error as previous error for next update cycle.
    err_prev[i] = err_curr[i];
  }

  // Set control efforts.
  setControlEfforts();
}

void resetEncoderCounts() {
  for(int i = 0; i < 4; i++) {
    // Set encoder counts to zero.
    enc_count[i] = 0;
  }
}

void setControlEfforts() {
  for(int i = 0; i < 4; i++) {
    // Set motor rotation direction.
    if(u[i] < 0) {
      digitalWrite(PINS_DIR[i], HIGH);
      is_negative[i] = true;
    } else {
      digitalWrite(PINS_DIR[i], LOW);
      is_negative[i] = false;
    }

    // Set motor speed magnitude
    analogWrite(PINS_PWM[i], abs(u[i]));
  }
}


void forceStop() {
  for(byte i = 0; i < 4; i++) {
    u[i] = 0;
    setpoints[i] = 0.0;
    err_sum[i] = 0.0;
    analogWrite(PINS_PWM[i], 0);
  }
}

void plowMove(bool move_up) {
  // Initialize local variables.
  byte pin_rly = (move_up ? PIN_RELAY_UP : PIN_RELAY_DOWN);
  byte pin_lim = (move_up ? PIN_LIMIT_UP : PIN_LIMIT_DOWN);

  // Start moving plow.
  if(move_up || !digitalRead(PIN_LIMIT_DOWN)) {
    digitalWrite(pin_rly, LOW);
  } else {
    plowStop();
  }
}

void plowStop() {
  digitalWrite(PIN_RELAY_DOWN, HIGH);
  digitalWrite(PIN_RELAY_UP, HIGH);
}


void serialEvent() {
  int data;
  
  while(Serial.available()) {
    data = Serial.read();
    if(data == LF) {
      processSerialData();
    } else {
      serial_data[serial_count] = data;
      serial_count++;
    }
  }
}

void processSerialData() {
  int temp = 0;
  byte index = 0;
  bool negative = false;

  if(serial_data[0] == '-') {
    index++;
    negative = true;
  }

  for(; index < serial_count; index++) {
    temp = (temp * 10) + serial_data[index] - 0x30;
  }
  serial_count = 0;

  if(negative) {
    setpoints[0] = float(-1 * temp);
    setpoints[1] = float(-1 * temp);
    setpoints[2] = float(-1 * temp);
    setpoints[3] = float(-1 * temp);
  } else {
    setpoints[0] = float(temp);
    setpoints[1] = float(temp);
    setpoints[2] = float(temp);
    setpoints[3] = float(temp);
  }

  if(temp == 0) {
    analogWrite(PINS_PWM[0], 0);
    analogWrite(PINS_PWM[1], 0);
    analogWrite(PINS_PWM[2], 0);
    analogWrite(PINS_PWM[3], 0);
  }
}


void encoder1ISR() {
  // Increment encoder count for encoder 1.
  enc_count[0]++;
}

void encoder2ISR() {
  // Increment encoder count for encoder 2.
  enc_count[1]++;
}

void encoder3ISR() {
  // Increment encoder count for encoder 3.
  enc_count[2]++;
}

void encoder4ISR() {
  // Increment encoder count for encoder 4.
  enc_count[3]++;
}

void timer1ExpirationISR() {
  // Set flag for timer 1 experiation.
  bitSet(flags, FLAG_TIMER1);
}

void timer3ExpirationISR() {
  // Set timeout.
  timeout = true;
}
