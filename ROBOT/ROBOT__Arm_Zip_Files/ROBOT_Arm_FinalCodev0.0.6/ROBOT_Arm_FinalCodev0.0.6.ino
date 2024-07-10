// to add location, make case in vSwitch_Loop
// 100 to stop
// 101 to start
// 200 stand control e.g. 215 for 15 packets
// Value of giMotor1,giMotor2 & giMotor3 should be equal to default position

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define MIN_PULSE_WIDTH   530
#define MAX_PULSE_WIDTH   1870
#define FREQUENCY         50

#define BASE_CONTROL 6
#define MID_CONTROL 7
#define UPDOWN_CONTROL 8

#define DEF_ANGLE_BASE 133
#define DEF_ANGLE_MID 160
#define DEF_ANGLE_UPDOWN 20

#define PICK_ANGLE_MID 140
#define PICK_ANGLE_UPDOWN 40

#define PLACE_ANGLE_MID1 175
#define PLACE_ANGLE_UPDOWN1 45

#define PLACE_ANGLE_MID2 155
#define PLACE_ANGLE_UPDOWN2 40

#define PLACE_ANGLE_MID3 120
#define PLACE_ANGLE_UPDOWN3 35

#define SUCTION_OFF_DELAY 500
#define SUCTION_ON_DELAY 500

#define SPEED_DELAY 15
#define DEF_OPR 60000

int giMax_pack = 243;


int giMotor1 = 133;
int giMotor2 = 160;
int giMotor3 = 20;

#define RXPIN  2
#define TXPIN  3

const int dirPin = 4;  // Direction
const int stepPin = 5; // Step

const int STEPS_PER_REV = 200;

int tiStand_pos;


SoftwareSerial mySerial =  SoftwareSerial(RXPIN, TXPIN);

int giConnected = 101;

void setup() {
  Serial.begin(9600);
  pinMode(RXPIN, INPUT);
  pinMode(TXPIN, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  mySerial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  vDefaultMotor();
}

void vDefaultMotor() {

  vMovemotor(UPDOWN_CONTROL, DEF_ANGLE_UPDOWN);
  vMovemotor(MID_CONTROL, DEF_ANGLE_MID);
  vMovemotor(BASE_CONTROL, DEF_ANGLE_BASE);
}

void vDefaultMotor1() {
  int tiloop1 = giMotor3;
  int tiloop2 = 0;
  bool tbflag = 0;
  while (true) {
    vReadEEPROM();
    if ((tbflag == 0) && (tiloop1>DEF_ANGLE_UPDOWN)) {
        tiloop1--;
        pwm.setPWM(UPDOWN_CONTROL, 0, ipulseWidth(tiloop1));
        if(tiStand_pos = giMax_pack){
          delay(SPEED_DELAY);
          }    
    }

    if ((tbflag == 1)&& (tiloop2<12) && (tiStand_pos < giMax_pack)) {
      digitalWrite(dirPin, LOW);
        for (int x = 0; x < ((STEPS_PER_REV / 4)); x++) {
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(500);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(500);
        }
        if (tiloop2 % 2 != 0) {
          tiStand_pos++;
          EEPROM[0] = tiStand_pos;
        }
        tiloop2++;    
    }
    tbflag = !tbflag;
    if (tiloop1 == 20)break;
  }

  giMotor3 = DEF_ANGLE_UPDOWN;
  vMovemotor(MID_CONTROL, DEF_ANGLE_MID);
  vMovemotor(BASE_CONTROL, DEF_ANGLE_BASE);
}

void vSuction_on() {
  digitalWrite(12, HIGH);
  digitalWrite(11, HIGH);
  delay(SUCTION_ON_DELAY);
}

void vSuction_off() {
  digitalWrite(12, LOW);
  digitalWrite(11, LOW);
  delay(SUCTION_OFF_DELAY);
}

void vMovemotor(int tiMotorpin, int tiDegree) { // to move motor to particular tiDegree
  unsigned long tuBegin_time = millis();
  unsigned long tuCurrent_time = millis();
  bool tbVal = 0;
  switch (tiMotorpin) {
    case BASE_CONTROL:
      if (tiDegree >= giMotor1) {
        for (int i = giMotor1 ; i <= tiDegree ; i++) {
          tbVal = 0;
          while (true) {
            tuCurrent_time = millis();
            if ((tuCurrent_time - tuBegin_time ) >= SPEED_DELAY) {
              pwm.setPWM(BASE_CONTROL, 0, ipulseWidth(i));
              tbVal = 1;
              tuBegin_time = tuCurrent_time;
            }
            if (tbVal == 1)break;
          }
        }
      } else if (tiDegree <= giMotor1) {
        for (int i = giMotor1 ; i >= tiDegree ; i--) {
          tbVal = 0;
          while (true) {
            tuCurrent_time = millis();
            if ((tuCurrent_time - tuBegin_time) >= SPEED_DELAY) {
              pwm.setPWM(BASE_CONTROL, 0, ipulseWidth(i));
              tbVal = 1;
              tuBegin_time = tuCurrent_time;
            }
            if (tbVal == 1)break;
          }
        }
      }
      giMotor1 = tiDegree;
      break;

    case MID_CONTROL:
      if (tiDegree >= giMotor2) {
        for (int i = giMotor2 ; i <= tiDegree ; i++) {
          tbVal = 0;
          while (true) {
            tuCurrent_time = millis();
            if ((tuCurrent_time - tuBegin_time) >= SPEED_DELAY) {
              pwm.setPWM(MID_CONTROL, 0, ipulseWidth(i));
              tbVal = 1;
              tuBegin_time = tuCurrent_time;
            }
            if (tbVal == 1)break;
          }
        }
      } else if (tiDegree <= giMotor2) {
        for (int i = giMotor2 ; i >= tiDegree ; i--) {
          tbVal = 0;
          while (true) {
            tuCurrent_time = millis();
            if ((tuCurrent_time - tuBegin_time) >= SPEED_DELAY) {
              pwm.setPWM(MID_CONTROL, 0, ipulseWidth(i));
              tbVal = 1;
              tuBegin_time = tuCurrent_time;
            }
            if (tbVal == 1)break;
          }
        }
      }
      giMotor2 = tiDegree;
      break;

    case UPDOWN_CONTROL:
      if (tiDegree >= giMotor3) {
        for (int i = giMotor3 ; i <= tiDegree ; i++) {
          tbVal = 0;
          while (true) {
            tuCurrent_time = millis();
            if ((tuCurrent_time - tuBegin_time) >= SPEED_DELAY) {
              pwm.setPWM(UPDOWN_CONTROL, 0, ipulseWidth(i));
              tbVal = 1;
              tuBegin_time = tuCurrent_time;
            }
            if (tbVal == 1) break;
          }
        }
      } else if (tiDegree <= giMotor3) {
        for (int i = giMotor3 ; i >= tiDegree ; i--) {
          tbVal = 0;
          while (true) {
            tuCurrent_time = millis();
            if ((tuCurrent_time - tuBegin_time) >= SPEED_DELAY) {
              pwm.setPWM(UPDOWN_CONTROL, 0, ipulseWidth(i));
              tbVal = 1;
              tuBegin_time = tuCurrent_time;
            }
            if (tbVal == 1)break;
          }
        }
      }
      giMotor3 = tiDegree;
      break;
  }
}

void vMotion1(int tiDegree) { //location to put the product
  vPick_Loop();
  vMovemotor(BASE_CONTROL, tiDegree);
  vMovemotor(UPDOWN_CONTROL, PLACE_ANGLE_UPDOWN1);
  vMovemotor(MID_CONTROL, PLACE_ANGLE_MID1 );
  vSuction_off();
  vDefaultMotor();
}

void vMotion2(int tiDegree) { //location to put the product
  vPick_Loop();
  vMovemotor(BASE_CONTROL, tiDegree);
  vMovemotor(UPDOWN_CONTROL, PLACE_ANGLE_UPDOWN2);
  vMovemotor(MID_CONTROL, PLACE_ANGLE_MID2 );
  vSuction_off();
  vDefaultMotor();
}

void vMotion3(int tiDegree) { //location to put the product
  vPick_Loop();
  vMovemotor(BASE_CONTROL, tiDegree);
  vMovemotor(UPDOWN_CONTROL, PLACE_ANGLE_UPDOWN3);
  vMovemotor(MID_CONTROL, PLACE_ANGLE_MID3 );
  vSuction_off();
  vDefaultMotor();
}

void vStepper() {
  for (int x = 0; x < (STEPS_PER_REV / 2); x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
}

void vReadEEPROM() {
  tiStand_pos  = EEPROM[0];
  if (tiStand_pos == 0) {
    tiStand_pos = 200;
    EEPROM[0] = tiStand_pos;
  }
}

void vcounterclockwise() {

  if (tiStand_pos > 200) {
    digitalWrite(dirPin, HIGH);
    vStepper();
    tiStand_pos--;
    EEPROM[0] = tiStand_pos;
    int xxx = EEPROM[0] ;
    Serial.print("UP: ");
    Serial.println(xxx);
  }
}

void vclockwise() {

  if (tiStand_pos < giMax_pack) {
    digitalWrite(dirPin, LOW);
    vStepper();
    tiStand_pos++;
    EEPROM[0] = tiStand_pos;

  }
}

void vPick_Loop() {     //location to pick the product
  int stand = EEPROM[0];
  vMovemotor(MID_CONTROL, PICK_ANGLE_MID );
  vMovemotor(UPDOWN_CONTROL, PICK_ANGLE_UPDOWN);
  vSuction_on();
  //vStandControl(x + 6 );
  vDefaultMotor1();

  vStandControl(stand - 1 );
}

void vMotor_Control() {
  unsigned long tuBeginTime = millis();
  String tsSub_S;
  mySerial.begin(9600);
  Serial.println("In which position Robot should move?");
  while (mySerial.available() == 0) {
    unsigned long tuCurrentTime = millis();
    if ((tuCurrentTime - tuBeginTime) > DEF_OPR) {
      vSwitch_Loop(99);
      break;
    }
  }

  String tsPos = mySerial.readString();
  mySerial.end();
  int tiIndex = tsPos.indexOf('/', 0);
  tsSub_S = tsPos.substring(0, tiIndex);
  int tiSub_S = tsSub_S.toInt();

  if (tiSub_S == 100) {
    giConnected = 100;
  } else if (tiSub_S < 100) {
    vSwitch_Loop(tiSub_S);
  } else if ((tiSub_S >= 200) && (tiSub_S <= giMax_pack)) {
    vStandControl(tiSub_S);
  }
}

void vStandControl(int pos) { // Stand Contolled by User
  vReadEEPROM();

  if (pos > tiStand_pos) {
    for (int i = (tiStand_pos); i < pos; i++) {
      vclockwise();
    }
  } else if (pos < tiStand_pos) {
    for (int i = (tiStand_pos); i > pos; i-- ) {
      vcounterclockwise();
    }
  }
}

void vSwitch_Loop(int tbVal) {
  switch (tbVal) {
    case 0:
      break;
    case 1:
      vMotion1(244);
      break;
    case 2:
      vMotion1(223);
      break;
    case 3:
      vMotion1(205);
      break;
    case 4:
      vMotion1(188);
      break;
    case 5:
      vMotion1(165);
      break;
    //    case 6:
    //      vMotion1(140);
    //      break;
    //    case 7:
    //      vMotion1(120);
    //      break;
    case 8:
      vMotion1(100);
      break;
    case 9:
      vMotion1(75);
      break;
    case 10:
      vMotion1(57);
      break;
    case 11:
      vMotion1(40);
      break;
    case 12:
      vMotion1(20);
      break;
    case 13:
      vMotion1(0);
      break;

    case 14:
      vMotion2(244);
      break;
    case 15:
      vMotion2(223);
      break;
    case 16:
      vMotion2(205);
      break;
    case 17:
      vMotion2(188);
      break;
    case 18:
      vMotion2(165);
      break;
    //    case 19:
    //      vMotion2(140);
    //      break;
    //    case 20:
    //      vMotion2(120);
    //      break;
    case 21:
      vMotion2(100);
      break;
    case 22:
      vMotion2(75);
      break;
    case 23:
      vMotion2(57);
      break;
    case 24:
      vMotion2(40);
      break;
    case 25:
      vMotion2(20);
      break;
    case 26:
      vMotion2(0);
      break;

    case 27:
      vMotion3(244);
      break;
    case 28:
      vMotion3(227);
      break;
    case 29:
      vMotion3(215);
      break;
    case 30:
      vMotion3(202);
      break;
    case 31:
      vMotion3(190);
      break;
    case 32:
      vMotion3(175);
      break;
    case 33:
      vMotion3(163);
      break;
    case 34:
      vMotion3(149);
      break;
    case 35:
      vMotion3(133);
      break;
    case 36:
      vMotion3(117);
      break;
    case 37:
      vMotion3(102);
      break;
    case 38:
      vMotion3(88);
      break;
    case 39:
      vMotion3(72);
      break;
    case 40:
      vMotion3(60);
      break;
    case 41:
      vMotion3(50);
      break;
    case 42:
      vMotion3(37);
      break;
    case 43:
      vMotion3(25);
      break;
    case 44:
      vMotion3(10);
      break;
    case 45:
      vMotion3(0);
      break;

    case 99 :
      //     vMotion2(150);
      break;
    default :
      break;
  }
}


void loop() {
  if (giConnected == 101) {
    vMotor_Control();
  } else if (giConnected == 100) {
    mySerial.begin(9600);
    while (mySerial.available() == 0) {}
    String tsPos = mySerial.readString();
    mySerial.end();
    int tiIndex = tsPos.indexOf('/', 0);
    String tsSub_S = tsPos.substring(0, tiIndex);
    if (tsSub_S == "101") {
      giConnected = 101;
    }
  }
}

int ipulseWidth(int angle) {
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4267);
  return analog_value;
}
