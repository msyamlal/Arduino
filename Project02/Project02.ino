int switchState = 0;

void setup(){
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(2, INPUT);
}

void loop(){
  switchState = digitalRead(2);
  if(switchState == LOW){
    //button is not pressed
    digitalWrite(3, HIGH); //green LED
    digitalWrite(4, LOW); //red LED
    digitalWrite(5, LOW); //red LED
  }
  else{ //button is pressed
    digitalWrite(3, LOW); //green LED
    digitalWrite(4, LOW); //red LED
    digitalWrite(5, HIGH); //red LED
    delay(250); //wait for a quarter second and toggle the LEDs
    digitalWrite(4, HIGH); //red LED
    digitalWrite(5, LOW); //red LED
    delay(250); //wait for a quarter second
  }
  
}
