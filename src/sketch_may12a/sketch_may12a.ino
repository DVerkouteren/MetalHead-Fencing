//Main code for MetalHead figure
#include <Servo.h>

//define pins
#define headPin 15
#define swordPin 13
int voltages[] = {0,0,0,0,0};
int indexa = 0;

//servos
Servo LegServo;
Servo FloorServo;
Servo HipServo;
Servo ArmServo;

#define servoFloorPin 2
#define servoLegPin 16
#define servoHipPin 0
#define servoArmPin 12

#define floorStartingPos 90
#define legStartingPos 76
#define hipStartingPos 78
#define armStartingPos 0

double floorPos = legStartingPos;
double legPos = legStartingPos;
double hipPos = legStartingPos;
double armPos = legStartingPos;

//joystick
#define Xaxis 32
#define Yaxis 33
#define Jswitch 25

int switchCount = 0;


//LED
#define LEDPin 19


void setup() {
  //define serial
  Serial.begin(9600);

  //define pinmodes
  //head/sword
  pinMode(headPin, OUTPUT);
  pinMode(swordPin, INPUT);
  
  //servos
  FloorServo.attach(servoFloorPin);
  LegServo.attach(servoLegPin);
  HipServo.attach(servoHipPin);
  ArmServo.attach(servoArmPin);

  FloorServo.write(floorStartingPos);
  LegServo.write(legStartingPos);
  HipServo.write(hipStartingPos);
  ArmServo.write(armStartingPos);

  //LED
  pinMode(LEDPin, OUTPUT);

  //joystick
  pinMode(Xaxis, INPUT);
  pinMode(Yaxis, INPUT);
  pinMode(Jswitch, INPUT);


  //init LED
  digitalWrite(LEDPin, 0);

  
  
}

void loop() {

  //create constant voltage in head
  analogWrite(headPin, 50);
  checkForWin();

  //get data from gyroscope

  //map data from gyroscope to position including min/max;


  //move servos

  //map and cap servo range
  int X = analogRead(Xaxis);
  int Y = analogRead(Yaxis);

  const int maxLegAngle = 75;
  const int minLegAngle = 20;

  floorPos = min(max((double) map(floorPos, 0, 1023, 0, 180), 0.0), 180.0);
  legPos = min(max((double) map(X, 0, 4095, minLegAngle, maxLegAngle), (double) minLegAngle), (double) maxLegAngle);
  hipPos = min(max((double) map(Y, 0, 4095, 0, 180), 0.0), 180.0);
  

  if(analogRead(Jswitch) == 0 && switchCount <= -100){
    switchCount = 15;
    armPos = 100;
  }
  if(switchCount <= 0){
    armPos = 90;
  }
  switchCount--;

  Serial.print("X axis:");
  Serial.print(X);
  Serial.print(", Leg Pos: ");
  Serial.print(legPos);
  Serial.print(", Y axis: ");
  Serial.print(Y);
  Serial.print(", Hip Pos: ");
  Serial.println(hipPos);
  // Serial.print(", SwitchCount: ");
  // Serial.print(switchCount);
  // Serial.print(", Switch ");
  // Serial.println(analogRead(Jswitch));



  //write to servo
  //FloorServo.write(floorPos);
  LegServo.write(legPos);
  HipServo.write(hipPos);
  ArmServo.write(armPos);
  delay(5);


}



void checkForWin(){
  
  
  if(indexa > 4){
    indexa = 0;
  }

  int voltage = analogRead(swordPin);
  voltages[indexa] = voltage;
  int sum = 0;

  for(int j = 0; j <= 4; j++){
    sum += voltages[j];
  }
  sum = sum/5;
  if(sum == 4095){ 
    digitalWrite(LEDPin, HIGH);  
  }else{
    digitalWrite(LEDPin, LOW);
  }
  indexa++;
}
