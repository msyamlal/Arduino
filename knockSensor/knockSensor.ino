int ledPin = 9;
int piezoPin = A5;
int threshold = 120;
int sensorValue = 0;
float ledValue = 0;

void setup(){
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(150);
  digitalWrite(ledPin, LOW);
  delay(150);
  digitalWrite(ledPin, HIGH);
  delay(150);
  digitalWrite(ledPin, LOW);
  delay(150);
  Serial.begin(9600);
}

void loop(){
  sensorValue = analogRead(piezoPin);
  Serial.println(sensorValue);
  if(sensorValue >= threshold){
    ledValue = 255;
  }
  analogWrite(ledPin, int(ledValue));
  ledValue = ledValue -0.05;
  if(ledValue <= 0) {
    ledValue = 0;
  }
  delay(100);
}
