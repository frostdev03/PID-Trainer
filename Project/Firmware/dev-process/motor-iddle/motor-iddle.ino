#define RPWM 2
#define LPWM 15
#define R_EN 27
#define L_EN 14

void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
}

void loop() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 255);

  delay(3000);

  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);

  analogWrite(RPWM, 255);
  analogWrite(LPWM, 0);

  for(int speed = 0; speed <= 255; speed++){
    analogWrite(RPWM, 0);
    analogWrite(LPWM, speed);
    delay(40);
  }

  for(int speed = 255; speed <= 0; speed--){
    analogWrite(RPWM, 0);
    analogWrite(LPWM, speed);
    delay(40);
  }

  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}
