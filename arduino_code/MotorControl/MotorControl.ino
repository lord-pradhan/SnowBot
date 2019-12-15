// MotorControl.ino
// Revised: 12/08/2019
// Device: Arduino MEGA 2560

// Library Imports
#include <Wire.h>
#include <TimerOne.h>
#include <TimerThree.h>

// Pin Definitions
const byte PINS_ENC[] = {2, 3, 18, 19};   // motor optical encoder inputs [1, 2, 3, 4]
const byte PINS_PWM[] = {5, 6, 7, 8};     // motor driver pwm outputs [1, 2, 3, 4]
const byte PINS_DIR[] = {22, 23, 24, 25}; // motor driver direction outputs [1, 2, 3, 4]
const byte PIN_WIFI_TX = 16;              // ESP8266 transmit
const byte PIN_WIFI_RX = 17;              // ESP8266 receive
const byte PIN_RELAY_UP = 26;             // linear actuator relay expansion output 1
const byte PIN_RELAY_DOWN = 27;           // linear actuator relay contraction output 2
//const byte PIN_LIMIT_UP = 28;             // linear actuator expansion limit switch input
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
int8_t speed_curr[] = {0, 0, 0, 0};

// Linear Actuator Timeout Fault
const long T_FAULT = 10000000L;  // linear actuator fault timeout period (us)
bool timeout = false;

// Motion Action Update Flags
volatile byte flags = 0x00;     // flag container
const byte FLAG_TIMER1 = 0;     // flag for Timer 1 expiration
const byte FLAG_PLOW_UP = 1;    // flag for plow raising command
const byte FLAG_PLOW_DOWN = 2;  // flag for plow lowering command
const byte FLAG_PLOW_STOP = 3;  // flag for plow stop command
const byte FLAG_LOS = 4;        // flag for Raspberry Pi loss of signal

bool plow_contracting = false;

// I2C Address
const byte ADDRESS = 0x08;
volatile byte los_count = 0;
const byte LOS_LIMIT = 24;      // 2s I2C idle timeout (based on T_SAMP)

// Serial Baud Rate
const long BAUD = 115200L;

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
//  pinMode(PIN_LIMIT_UP, INPUT);     // set plow expansion limit switch as input
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

  // Initialize & set up I2C bus.
  Wire.begin(ADDRESS);
  Wire.onReceive(i2cReceiveHandler);
  Wire.onRequest(i2cRequestHandler);
  
  // Initialize serial.
  Serial.begin(BAUD);
  
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
}


void loop() {
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
//    forceStop();
    plowMove(true);                 // raise plow
  }

  // Plow lower action.
  if(bitRead(flags, FLAG_PLOW_DOWN)) {
    bitClear(flags, FLAG_PLOW_DOWN);  // clear plow lowering command flag
//    forceStop();
    plowMove(false);                  // lower plow
  }

  // Plow stop action.
  if(plow_contracting && digitalRead(PIN_LIMIT_DOWN)) {
    plow_contracting = false;
    plowStop();
  }

  // Raspberry Pi LOS action.
  if(bitRead(flags, FLAG_LOS)) {
    bitClear(flags, FLAG_LOS);
    los_count = 0;
    forceStop();
//    plowStop();
  }
}


void updatePID() {
  // Declare local variables.
  float err_curr[4];
  float err_diff[4];
  float up[4], ui[4], ud[4];
  int u_prev[4], slew[4];

  // Store encoder counts locally.
  speed_curr[0] = enc_count[0];
//  speed_curr[1] = enc_count[0];//enc_count[1]; //  BROKEN ENCODER, MIMIC OTHER WHEEL
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
  bool limit_hit = false;
  byte pin_rly = (move_up ? PIN_RELAY_UP : PIN_RELAY_DOWN);
  byte pin_opp = (!move_up ? PIN_RELAY_UP : PIN_RELAY_DOWN);
//  byte pin_lim = PIN_LIMIT_DOWN;//(move_up ? PIN_LIMIT_UP : PIN_LIMIT_DOWN);
  
  // Start moving plow.
  digitalWrite(pin_rly, LOW);
  digitalWrite(pin_opp, HIGH);

  // Timer function changed for limit switches.
  // "Up" limit switch is built-in, not explicitly checked.
  
  // Set up fault timeout timer.
//  timeout = false;    // initialize timeout flag
//  Timer3.initialize(T_FAULT);
//  Timer3.attachInterrupt(timer3ExpirationISR);
  if(move_up) {
    plow_contracting = false; // not needed
//    delay(5000);
  } else {
    plow_contracting = true;
  }
//  // Wait for limit switch or fault timeout.
//  while(!limit_hit && !timeout) {
//    limit_hit = !digitalRead(pin_lim);
//    serialEvent();
//    if(bitRead(flags, FLAG_PLOW_STOP)) {
//      bitClear(flags, FLAG_PLOW_STOP);
//      break;
//    }
//  }
  
  // Stop moving plow.
//  if(move_up) {
//    digitalWrite(pin_rly, HIGH);
//  }

  // Stop Timer 3 interrupts.
//  Timer3.detachInterrupt();
}

void plowStop() {
  digitalWrite(PIN_RELAY_UP ,HIGH);
  digitalWrite(PIN_RELAY_DOWN, HIGH);
}


void i2cReceiveHandler(int numBytes) {
  // Reset Raspberry Pi LOS counter
  los_count = 0;
  int8_t sp_read;
  byte plow_action;
  
  if(Wire.available() == 5) {
    plow_action = Wire.read();
    if(plow_action == 1) {
      bitSet(flags, FLAG_PLOW_DOWN);
    } else if(plow_action == 2) {
      bitSet(flags, FLAG_PLOW_UP);
    }
    for(byte i = 0; i < 4; i++) {
      sp_read = Wire.read();
      setpoints[i] = float(sp_read);
    }
  } else {
    while(Wire.available()) {
      Wire.read();
    }
  }
}

void i2cRequestHandler() {
  byte data[12];
  int8_t temp;

  // Reset Raspberry Pi LOS counter
  los_count = 0;

  for(byte i = 0; i < 4; i++) {
    temp = speed_curr[i];
    data[i] = byte(temp);
  }
  
  for(byte i = 0; i < 4; i++) {
    temp = u[i] & 0xff;
    data[2*i+4] = byte(temp);
    temp = (u[i] >> 8) & 0xff;
    data[2*i+5] = byte(temp);
  }
  Wire.write(data, 12);
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

  // Plow Commands
  if(serial_data[0] == 'u' || serial_data[0] == 'U') {
    bitSet(flags, FLAG_PLOW_UP);
    serial_count = 0;
    return;
  }
  if(serial_data[0] == 'd' || serial_data[0] == 'D') {
    bitSet(flags, FLAG_PLOW_DOWN);
    serial_count = 0;
    return;
  }
  if(serial_data[0] == 's' || serial_data[0] == 'S') {
    bitSet(flags, FLAG_PLOW_STOP);
    serial_count = 0;
    return;
  }

  if(serial_data[0] == '-') {
    index++;
    negative = true;
  }

  // Speed Commands
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
    forceStop();
  }
}


void encoder1ISR() {
  // Incremen   t encoder count for encoder 1.
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
  los_count++;
  if(los_count >= LOS_LIMIT) {
    bitSet(flags, FLAG_LOS);
  }
}

void timer3ExpirationISR() {
  // Set timeout.
  timeout = true;
}
