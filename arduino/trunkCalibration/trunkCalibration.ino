#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HerkulexServo.h>

#define PIN_SW_RX 10
#define PIN_SW_TX 11

SoftwareSerial   servo_serial(PIN_SW_RX, PIN_SW_TX);
HerkulexServoBus herkulex_bus(servo_serial);
HerkulexServo    *servo [8];
uint16_t servo_positions[8];

unsigned long last_update = 0;
unsigned long now = 0;
uint8_t cableId = 0;

const uint8_t NB_CABLES = 8;
const uint16_t SMIN = 297;
const uint16_t SMAX = 666;
const uint16_t LMIN = 974;
const uint16_t LMAX = 297;
// const uint16_t INIT[NB_CABLES] = {639, 564, 389, 564, 680, 605, 805, 605};
const uint16_t INIT[NB_CABLES] = {666, 564, 389, 564, 563, 527, 805, 605}; 
// const uint16_t SINIT = SMIN+abs(SMAX-SMIN)/4;
// const uint16_t LINIT = LMIN-abs(LMIN-LMAX)/4;
const uint16_t RANGE = 39;

const uint8_t NB_ACTIONS = 100;
uint8_t actions[NB_ACTIONS] = {5, 12, 12, 12, 12, 12, 7, 7, 7, 3, 3, 3, 3, 13, 13, 13, 5, 5, 5, 5, 4, 4, 4, 0, 0, 0, 2, 13, 13, 13, 13, 6, 4, 4, 4, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13};
//uint8_t actions[NB_ACTIONS] = {5,12,12,12,3,3,7,7,7,13,13,7,12,12,5,5,5,9,9,12,12,12,12,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9}; // CI4OB
//uint8_t actions[NB_ACTIONS] = {5,12,12,12,3,3,7,7,7,13,13,7,7,7,1,1,1,1,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14}; // 4YADE

void translate_actions(){
  for(uint8_t i = 0; i < NB_ACTIONS; i++) {
    if (actions[i] < 8)
      actions[i] = (actions[i] + 4) % 8;
    else
      actions[i] = (actions[i] + 4) % 8 + 8;
  }
}

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
  position = (i<4) ? constrain(position, SMIN, SMAX) : constrain(position, LMAX, LMIN);
  servo[i]->setPosition(position, 100);
  servo_positions[i] = position;
}

void set_position_low(uint16_t i){
  uint16_t position = (i<4) ? SMIN : LMIN;
  servo[i]->setPosition(position, 100);
  servo_positions[i] = position;
}

void set_position_reset(uint16_t i){ /*TODO for horizontal position */
  uint16_t position = INIT[i];
  servo[i]->setPosition(position, 100);
  servo_positions[i] = position;
}

void pull_cable(uint16_t i){
  uint16_t position = servo_positions[i];
  position = (i < 4) ? position+RANGE : position-RANGE;
  set_position(i, position);
}

void release_cable(uint16_t i){
  uint16_t position = servo_positions[i];
  position = (i < 4) ? position-RANGE : position+RANGE;
  set_position(i, position);
}


void action_to_command(uint16_t action) {
    uint16_t cableId;
    switch(action) {
        case 0:
            cableId = 0;
            pull_cable(cableId);
            break;
        case 1:
            cableId = 1;
            pull_cable(cableId);
            break;
        case 2:
            cableId = 2;
            pull_cable(cableId);
            break;
        case 3:
            cableId = 3;
            pull_cable(cableId);
            break;
        case 4:
            cableId = 4;
            pull_cable(cableId);
            break;
        case 5:
            cableId = 5;
            pull_cable(cableId);
            break;
        case 6:
            cableId = 6;
            pull_cable(cableId);
            break;
        case 7:
            cableId = 7;
            pull_cable(cableId);
            break;
        case 8:
            cableId = 0;
            release_cable(cableId);
            break;
        case 9:
            cableId = 1;
            release_cable(cableId);
            break;
        case 10:
            cableId = 2;
            release_cable(cableId);
            break;
        case 11:
            cableId = 3;
            release_cable(cableId);
            break;
        case 12:
            cableId = 4;
            release_cable(cableId);
            break;
        case 13:
            cableId = 5;
            release_cable(cableId);
            break;
        case 14:
            cableId = 6;
            release_cable(cableId);
            break;
        case 15:
            cableId = 7;
            release_cable(cableId);
            break;
    }
}

void simulate_policy(){
  for(uint8_t i = 0; i<NB_ACTIONS; i++) {
    Serial.print("Step ");
    Serial.print(i);
    Serial.print(" Action ");
    Serial.print(actions[i]);
    Serial.print(" Time ");
    Serial.println(millis());
    action_to_command(actions[i]);
    delay(500);
  }
}

void setup() {
  Serial.begin(115200);
  servo_serial.begin(115200);
  delay(500);
  for(uint8_t i = 0; i<NB_CABLES; i++) {
    servo[i] = new HerkulexServo(herkulex_bus, i);
    servo[i]->setTorqueOn();  // turn power on
    Serial.print(i);
    Serial.print(") ");
    uint16_t init = (i<4) ? SMIN : LMIN;
    servo[i]->setPosition(init);
    servo_positions[i] = init;
    Serial.print(init);
    Serial.print(" ");
  }
  Serial.println();
  delay(1000);
  translate_actions();
}

void loop() {
  herkulex_bus.update();

  if (Serial.available() > 0) {
    char c = Serial.read();

    // Serial.print("Selected cable: ");
    // Serial.println(cableId);
    // displayCables_byArray();

    String ids = "012345678";
    int isid = ids.indexOf(c);
    if (isid > -1) {
        cableId = c-'0';
        Serial.print("Selected cable: ");
        Serial.println(cableId);
    }
    else if(c == 'p') {
        // pull cable (plus)
        pull_cable(cableId);
        Serial.print("+ ");
    }
    else if(c=='m') {
        // release cable (minus)
        release_cable(cableId);
        Serial.print("- ");
    }
    else if(c=='a') {
        displayCables_byArray();
    }
    else if(c=='l') {
        for(uint8_t i = 0; i<NB_CABLES; i++) {
          set_position_low(i);
        }
    }
    else if(c=='r') {
        for(uint8_t i = 0; i<NB_CABLES; i++) {
          set_position_reset(i);
        }
    }
    else if(c=='s') {
      Serial.println();
        simulate_policy();
    }
  }
}
