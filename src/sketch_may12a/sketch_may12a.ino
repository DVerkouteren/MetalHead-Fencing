//Main code for MetalHead figure

//define pins
#define headPin 15
#define swordPin 13

//servos
#define servoFloorPin 3
#define servoLegPin 3
#define servoHipPin 3
#define servoArmPin 3

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
  pinMode(servoFloorPin, OUTPUT);
  pinMode(servoLegPin, OUTPUT);
  pinMode(servoHipPin, OUTPUT);
  pinMode(servoArmPin, OUTPUT);

  //LED
  pinMode(LEDPin, OUTPUT);


  //init LED
  digitalWrite(LEDPin, 0);


}

void loop() {

  //create constant voltage in head
  digitalWrite(headPin, HIGH);
  checkForWin();

}

void checkForWin(){
  int voltage = digitalRead(swordPin);
  if(voltage == HIGH){ 
    digitalWrite(LEDPin, HIGH);
    Serial.println("on");
    Serial.println(voltage);
  }else{
    digitalWrite(LEDPin, 0);
    Serial.println(voltage);
  }
}
