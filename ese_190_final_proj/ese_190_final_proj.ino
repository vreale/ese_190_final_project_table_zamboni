#define trigPin 13
#define echoPin 4
const int potPin = A0;  // Analog input pin that the potentiometer is attached to
const int Mleft = 6; // Analog output pin that the Motor is attached to
const int Mright = 5;

int sensorValue = 0;        // value read from the pot
int LoutputValue = 0;
int leftOrRight = 0;        //left = 0, right = 1
void setup () {
pinMode (trigPin , OUTPUT );
pinMode (echoPin , INPUT );
Serial.begin (9600);
// analogWrite(Mleft, 300);   // Left Motor -> LOW
// analogWrite(Mright, 300);
}

void loop () {
long duration , distance;
digitalWrite (trigPin , LOW ); // start trig at 0
delayMicroseconds (2);
digitalWrite (trigPin , HIGH ); //The rising edge of trig pulse
delayMicroseconds (10); // decides duration of trig pulse
digitalWrite (trigPin , LOW ); //falling edge of the trig pulse
// NOTE: echo pin reads HIGH till it receives the reflected signal
duration = pulseIn (echoPin , HIGH ); // Reading the duration for which echoPin was HIGH gives
//you the time the sensor receives a reflected signal at the echo pin
distance = (duration / 2) / 29.1; //Calculate the distance of the reflecting surface in cm
//
Serial.print("distance:");
Serial.println(distance);

 if(distance > 8) {
  //STOP
  analogWrite(Mleft, 0);   // Left Motor -> LOW
  analogWrite(Mright, 0);  // Right Motor -> LOW
  delay(500);
  if(leftOrRight == 0) {
    leftOrRight = 1;
    analogWrite(Mleft, 0);   // Left Motor -> LOW
    analogWrite(Mright, 100);  // RIGHT Motor -> HIGH (Speed set by Pot)
    delay(2200);
  } else {
    leftOrRight = 0; 
    analogWrite(Mleft, 110);   // Left Motor -> LOW
    analogWrite(Mright, 0);  // RIGHT Motor -> HIGH (Speed set by Pot)
    delay(2200);
  }
 } else {
   analogWrite(Mleft, 110);   // Left Motor -> LOW
   analogWrite(Mright, 80);
   delay(20);
 }
// delay(2000);
}
