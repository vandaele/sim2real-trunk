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
const uint16_t INIT[NB_CABLES] = {828, 774, 1000, 758, 456, 370, 400, 360};
const uint16_t RANGE = 40;

SoftwareSerial   servo_serial(PIN_SW_RX, PIN_SW_TX);
HerkulexServoBus herkulex_bus(servo_serial);
HerkulexServo    *servo [NB_CABLES];
uint16_t servo_positions[NB_CABLES];

unsigned long last_update = 0;
unsigned long now = 0;
uint8_t cableId = 0;

const uint8_t PLAYTIME = 9; // playtime = time_ms / 11.2
uint16_t init_time = 0;


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
  position = (i < 4) ? position-RANGE : position+RANGE;
  set_position(i, position);
}

void release_cable(uint16_t i){
  uint16_t position = servo_positions[i];
  position = (i < 4) ? position+RANGE : position-RANGE;
  set_position(i, position);
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

// the setup function runs once when you press reset or power the board
void setup(){
  Serial.begin(115200);
  Serial.println("Trunk Actuation Firmware");
  Serial.println("--------------------------");
  servo_serial.begin(115200);
  delay(500);
  for(uint8_t i = 0; i<NB_CABLES; i++) {
    servo[i] = new HerkulexServo(herkulex_bus, i);
    servo[i]->setTorqueOn();  // turn power on
  }
  Serial.println();
  delay(1000);
  Serial.println("Setup done");
}

// the loop function runs over and over again forever
void loop(){
  herkulex_bus.update();

  if (Serial.available()) {
    String a = Serial.readStringUntil('\n');  // read the input until newline
    if (a == "i") {
      for (uint8_t i = 0; i < NB_CABLES; i++) {
        set_position_init(i);
      }
      Serial.println("Initial position done");
    }
    else if (a == "l") {
      for (uint8_t i = 0; i < NB_CABLES; i++) {
        set_position_low(i);
      }
      Serial.println("Low position done");
    }
    else {
      uint16_t action = a.toInt();  // convert the input to an integer
      action_to_command(action);
    }
  };
}