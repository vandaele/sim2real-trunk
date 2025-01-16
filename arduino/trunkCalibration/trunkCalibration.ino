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
const uint16_t INIT[NB_CABLES] = {472, 469, 374, 479, 812, 780, 945, 781}; 
// const uint16_t SINIT = SMIN+abs(SMAX-SMIN)/4;
// const uint16_t LINIT = LMIN-abs(LMIN-LMAX)/4;
const uint16_t RANGE = 39;

const unsigned long DELAY = 50;
const uint8_t PLAYTIME = 100;

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
  servo[i]->setPosition(position, 100);
  servo_positions[i] = position;
}

void set_position_init(uint16_t i){ /*TODO for horizontal position */
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

void setup_motors() {
  Serial.begin(115200);
  servo_serial.begin(115200);
  delay(500);
  Serial.println("Setup done");
  for(uint8_t i = 0; i<NB_CABLES; i++) {
    servo[i] = new HerkulexServo(herkulex_bus, i);
    servo[i]->setTorqueOn();  // turn power on
    // Serial.print(i);
    // Serial.print(") ");
    // uint16_t init = (i<4) ? SMIN : LMIN;
    // servo[i]->setPosition(init, PLAYTIME); // set position to low position
    // servo_positions[i] = init;
    // Serial.print(init);
    // Serial.print(" ");
  }
  Serial.println();
  delay(1000);
  // translate_actions(); // adapt the fixed actions array to servo motors control
}

void loop_motors_gui() {
  herkulex_bus.update();

  if (Serial.available() > 0) {
    char c = Serial.read();
    Serial.print(c);

    // Serial.print("Selected cable: ");
    // Serial.println(cableId);
    // displayCables_byArray();

    String ids = "012345678";
    int isid = ids.indexOf(c);
    if (isid > -1) {
        // SELECT CABLE
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
    else if(c=='f') {
        displayCables_byFeedback();
    }
    else if(c=='l') {
        for(uint8_t i = 0; i<NB_CABLES; i++) {
          set_position_low(i);
        }
    }
    else if(c=='i') {
        for(uint8_t i = 0; i<NB_CABLES; i++) {
          set_position_init(i);
        }
        Serial.println("Initial position done");
    }
   else{
      Serial.print("ERROR - Command not recognised");
   }
  }
}

// the setup function runs once when you press reset or power the board
void setup(){
  setup_motors();
}

// the loop function runs over and over again forever
void loop(){
  loop_motors_gui();
}