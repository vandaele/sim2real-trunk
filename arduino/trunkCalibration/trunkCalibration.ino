#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HerkulexServo.h>

#define PIN_SW_RX 10
#define PIN_SW_TX 11

const uint8_t NB_CABLES = 8;
const uint16_t SMIN = 280;
const uint16_t SMAX = 640;
const uint16_t LMIN = 1000;
const uint16_t LMAX = 280;
// const uint16_t INIT[NB_CABLES] = {930, 766, 1000, 796, 600, 492, 560, 440};
const uint16_t INIT[NB_CABLES] = {828, 762, 1000, 798, 600, 492, 560, 440};
const uint16_t RANGE_HIGH = 40;
const uint16_t RANGE_LOW = 2;
const uint8_t PLAYTIME = 9; // playtime = time_ms / 11.2

SoftwareSerial   servo_serial(PIN_SW_RX, PIN_SW_TX);
HerkulexServoBus herkulex_bus(servo_serial);
HerkulexServo    *servo [NB_CABLES];
uint16_t servo_positions[NB_CABLES];

unsigned long last_update = 0;
unsigned long now = 0;
String cableIds = "";
uint8_t cableId = 0;
uint16_t range = RANGE_HIGH;
bool torqueOnForAll = true;
bool torqueOn[NB_CABLES] = {true, true, true, true, true, true, true, true};

const uint8_t NB_ACTIONS = 100;
// AKLWI
const uint8_t actions[NB_ACTIONS] = {5, 12, 12, 12, 7, 7, 3, 3, 7, 7, 13, 13, 11, 11, 11, 11, 1, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14};
// X0HBR
// const uint8_t actions[NB_ACTIONS] = {5, 12, 12, 3, 7, 7, 7, 7, 7, 13, 12, 8, 8, 1, 1, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8};
const unsigned long DELAY = 100;
uint16_t step = 0;
uint16_t init_time = 0;



void displayCables_byFeedback(){
  for(uint8_t i = 0; i < NB_CABLES; i++) {
        Serial.print(i);
        Serial.print(") ");
        Serial.print(servo[i]->getPosition());
        Serial.print(" ");
        }
  Serial.println();
}

void displayCables_byArray(){
  for(uint8_t i = 0; i < NB_CABLES; i++) {
        Serial.print(i);
        Serial.print(") ");
        Serial.print(servo_positions[i]);
        Serial.print(" ");
        }
  Serial.println();
}

void set_position(uint16_t i, uint16_t position){
  position = (i<4) ? constrain(position, LMAX, LMIN) : constrain(position, SMIN, SMAX);
  servo[i]->setPosition(position, PLAYTIME);
  servo_positions[i] = position;
}

void set_position_low(uint16_t i){
  uint16_t position = (i<4) ? LMIN : SMIN;
  servo[i]->setPosition(position, 255);
  servo_positions[i] = position;
}

void set_position_init(uint16_t i){
  uint16_t position = INIT[i];
  servo[i]->setPosition(position, 255);
  servo_positions[i] = position;
}

void pull_cable(uint16_t i){
  uint16_t position = servo_positions[i];
  position = (i < 4) ? position-range : position+range;
  set_position(i, position);
}

void release_cable(uint16_t i){
  uint16_t position = servo_positions[i];
  position = (i < 4) ? position+range : position-range;
  set_position(i, position);
}

void toggle_torque(uint16_t i){
  torqueOn[i] = !torqueOn[i];
  if(torqueOn[i])
    servo[i]->setTorqueOn();
  else
    servo[i]->setTorqueOff();
}

void action_to_command(uint16_t action) {
    if(action <0 || action > 15){
        Serial.println("ERROR: action value not in range [0-15]");
        return;
    }
    uint16_t cableId = action%8;
    if(action < 8)
        pull_cable(cableId);
    else
        release_cable(cableId);
}

void step_policy(uint16_t i){
  Serial.print("Step: ");
  Serial.print(i);
  Serial.print(" - Action: ");
  Serial.print(actions[i]);
  Serial.print(" - Time passed: ");
  Serial.print(millis() - init_time);
  Serial.println(" ms");
  action_to_command(actions[i]);
}

void simulate_policy(){
  step = 0;
  init_time = millis();
  while( step < NB_ACTIONS) {
    now = millis();
    if ( (now - last_update) >= DELAY) {
      step_policy(step);
      last_update = now;
      step++;
    }
  }
}

void switch_range_value() {
  range = (range == RANGE_HIGH) ? RANGE_LOW : RANGE_HIGH;
}

// the setup function runs once when you press reset or power the board
void setup(){
  for(uint8_t i = 0; i < NB_CABLES; i++){
      cableIds.concat(i);
  }
  Serial.begin(115200);
  servo_serial.begin(115200);
  delay(500);
  for(uint8_t i = 0; i<NB_CABLES; i++) {
    servo[i] = new HerkulexServo(herkulex_bus, i);
    if(torqueOn[i]) { servo[i]->setTorqueOn(); }
  }
  delay(1000);
  Serial.println("Setup done");
}

// the loop function runs over and over again forever
void loop(){
  herkulex_bus.update();

  if (Serial.available() > 0) {
    char c = Serial.read();
    Serial.print(c);

    int isid = cableIds.indexOf(c);
    if (isid > -1) {
        // SELECT CABLE
        cableId = c-'0';
        Serial.print("Selected cable: ");
        Serial.println(cableId);
    }
    else if(c == 'p') {
        pull_cable(cableId);
        Serial.print("+ ");
    }
    else if(c=='r') {
        release_cable(cableId);
        Serial.print("- ");
    }
    else if(c=='a') {     
        displayCables_byArray();
    }
    else if(c=='f') {
        displayCables_byFeedback();
    }
    else if(c=='o') {
        toggle_torque(cableId);
        Serial.print("motor: ");
        Serial.println( torqueOn[cableId] ? "ON" : "OFF");
    }
    else if(c=='O') {
        for(uint8_t i = 0; i<NB_CABLES; i++) {
          torqueOn[i] = torqueOnForAll;
          toggle_torque(i);
        }
        torqueOnForAll = !torqueOnForAll;
        Serial.print("All motors ");
        Serial.println(torqueOnForAll ? "ON" : "OFF");
    }
    else if(c=='l') {
        set_position_low(cableId);
    }
    else if(c=='L') {
        for(uint8_t i = 0; i<NB_CABLES; i++) {
          set_position_low(i);
        }
        Serial.println("Low position done");
    }
    else if(c=='i') {
        set_position_init(cableId);
    }
    else if(c=='I') {
      step = 0;
        for(uint8_t i = 0; i<NB_CABLES; i++) {
          set_position_init(i);
        }
        Serial.println("Initial position done");
    }
    else if(c=='s') {
      step_policy(step);
      step++;
    }
    else if(c=='S') {
      simulate_policy();
    }
    else if(c=='c') {
      switch_range_value();
      Serial.print("Cable movement range is now ");
      Serial.println(range);
    }
    else{
      Serial.println("ERROR - Command not recognised");
    }
  }
}