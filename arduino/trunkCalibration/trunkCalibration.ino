#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HerkulexServo.h>

#define PIN_SW_RX 10
#define PIN_SW_TX 11

const uint8_t NB_CABLES = 8;
const uint16_t SMIN = 297;
const uint16_t SMAX = 666;
const uint16_t LMIN = 974;
const uint16_t LMAX = 297;
const uint16_t INIT[NB_CABLES] = {472, 469, 374, 479, 812, 780, 945, 781}; 
const uint16_t RANGE = 39;
const uint8_t PLAYTIME = 45; // playtime = time_ms / 11.2

SoftwareSerial   servo_serial(PIN_SW_RX, PIN_SW_TX);
HerkulexServoBus herkulex_bus(servo_serial);
HerkulexServo    *servo [NB_CABLES];
uint16_t servo_positions[NB_CABLES];

unsigned long last_update = 0;
unsigned long now = 0;
String cableIds = "";
uint8_t cableId = 0;

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
  servo[i]->setPosition(position, PLAYTIME);
  servo_positions[i] = position;
}

void set_position_low(uint16_t i){
  uint16_t position = (i<4) ? SMIN : LMIN;
  servo[i]->setPosition(position, 200);
  servo_positions[i] = position;
}

void set_position_init(uint16_t i){
  uint16_t position = INIT[i];
  servo[i]->setPosition(position, 200);
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
    servo[i]->setTorqueOn();  // turn power on
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
    else if(c=='m') {
        release_cable(cableId);
        Serial.print("- ");
    }
    else if(c=='a') {     
        displayCables_byArray();
    }
    else if(c=='f') {
        displayCables_byFeedback();
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
        for(uint8_t i = 0; i<NB_CABLES; i++) {
          set_position_init(i);
        }
        Serial.println("Initial position done");
    }
    else{
      Serial.println("ERROR - Command not recognised");
    }
  }
}