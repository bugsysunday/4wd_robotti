#include <IRremote.h>
#include <IRremoteInt.h>

#include <Servo.h>

//Pins for motor A 
const int pinA1 = 2;
const int pinA2 = 3; 
const int enableA = 1;
//Pins for motor B 
const int pinB1 = 4;
const int pinB2 = 5;
const int enableB = 6;
//Pins for ultrasonic sensor
const int trigger=8;
const int echo=9;

int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
char choice;
  
//Pin for IR control
int receiver = 10; // pin 1 of IR receiver to Arduino digital pin 12
IRrecv irrecv(receiver);           // create instance of 'irrecv'
decode_results results; 
char contcommand;
int modecontrol=0;
int power=0;

const int distancelimit = 20; //Distance limit for obstacles in front           
const int sidedistancelimit = 20; //Minimum distance in cm to obstacles at both sides (the robot will allow a shorter distance sideways)

int distance;
int numcycles;
char turndirection; //Gets 'l', 'r' or 'f' depending on which direction is obstacle free
const int turntime = 400; //Time the robot spends turning (miliseconds)
int thereis;
Servo head;

void setup(){
   /* initialize serial port */
  head.attach(7);
  head.write(80);
  irrecv.enableIRIn(); // Start the IR receiver
  pinMode (enableA, OUTPUT);
  pinMode(pinA1, OUTPUT); 
  pinMode(pinA2, OUTPUT); 
  pinMode (enableB, OUTPUT);
  pinMode(pinB1, OUTPUT); 
  pinMode(pinB2, OUTPUT);
  pinMode(trigger,OUTPUT);
  pinMode(echo,INPUT);
  digitalWrite(trigger,LOW);
}

void go(){ 
   forward();
}

void backwards(){
  backward();
}

int watch(){
  //Trigger a HIGH pulse for 2 or more microseconds
 //Give a short LOW pulse before sending a HIGH one
 digitalWrite (trigger, LOW);
 delayMicroseconds(2);

 digitalWrite(trigger, HIGH);
 delayMicroseconds(10);

 digitalWrite (trigger, LOW);

 //Now, lets read the read the bounced wave
 long duration = pulseIn(echo, HIGH);

 //calculate the distance
 long distance = microsecondsToCentimeters(duration);
  return distance;
}

long microsecondsToCentimeters (long microseconds) {
 // The speed of sound is 340 m/s or 29 microseconds per centimeter
 // The ping travels forth and back, so, the distance is half the distance traveled
 return microseconds / 29 / 2;
}

void turnleft(int t){
  left();  
  delay(t);
}

void turnright(int t){
  right();
  delay(t);
}  

void stopmove(){
  coast();
} 





void motorAforward() {
 digitalWrite (pinA1, HIGH);
 digitalWrite (pinA2, LOW);
}
void motorBforward() {
 digitalWrite (pinB1, LOW);
 digitalWrite (pinB2, HIGH);
}
void motorAbackward() {
 digitalWrite (pinA1, LOW);
 digitalWrite (pinA2, HIGH);
}
void motorBbackward() {
 digitalWrite (pinB1, HIGH);
 digitalWrite (pinB2, LOW);
}
void motorAstop() {
 digitalWrite (pinA1, HIGH);
 digitalWrite (pinA2, HIGH);
}
void motorBstop() {
 digitalWrite (pinB1, HIGH);
 digitalWrite (pinB2, HIGH);
}
void motorAcoast() {
 digitalWrite (pinA1, LOW);
 digitalWrite (pinA2, LOW);
}
void motorBcoast() {
 digitalWrite (pinB1, LOW);
 digitalWrite (pinB2, LOW);
}
void motorAon() {
 digitalWrite (enableA, HIGH);
}
void motorBon() {
 digitalWrite (enableB, HIGH);
}
void motorAoff() {
 digitalWrite (enableA, LOW);
}
void motorBoff() {
 digitalWrite (enableB, LOW);
}
// Movement functions
void forward () {
 motorAforward();
 motorBforward();

}
void backward () {
 motorAbackward();
 motorBbackward();

}
void right () {
 motorAbackward();
 motorBforward();

}
void left () {
 motorAforward();
 motorBbackward();

}
void coast () {
 motorAcoast();
 motorBcoast();

}
void breakRobot () {
 motorAstop();
 motorBstop();

}
void disableMotors() {
 motorAoff();
 motorBoff();
}
void enableMotors() {
 motorAon();
 motorBon();
}







void watchsurrounding(){ //Meassures distances to the right, left, front, left diagonal, right diagonal and asign them in cm to the variables rightscanval, 
                         //leftscanval, centerscanval, ldiagonalscanval and rdiagonalscanval (there are 5 points for distance testing)
  centerscanval = watch();
  if(centerscanval<distancelimit){stopmove();}
  head.write(120);
  delay(100);
  ldiagonalscanval = watch();
  if(ldiagonalscanval<distancelimit){stopmove();}
  head.write(160); //Didn't use 180 degrees because my servo is not able to take this angle
  delay(300);
  leftscanval = watch();
  if(leftscanval<sidedistancelimit){stopmove();}
  head.write(120);
  delay(100);
  ldiagonalscanval = watch();
  if(ldiagonalscanval<distancelimit){stopmove();}
  head.write(80); //I used 80 degrees because its the central angle of my 160 degrees span (use 90 degrees if you are moving your servo through the whole 180 degrees)
  delay(100);
  centerscanval = watch();
  if(centerscanval<distancelimit){stopmove();}
  head.write(40);
  delay(100);
  rdiagonalscanval = watch();
  if(rdiagonalscanval<distancelimit){stopmove();}
  head.write(0);
  delay(100);
  rightscanval = watch();
  if(rightscanval<sidedistancelimit){stopmove();}

  head.write(80); //Finish looking around (look forward again)
  delay(300);
}

char decide(){
  watchsurrounding();
  if (leftscanval>rightscanval && leftscanval>centerscanval){
    choice = 'l';
  }
  else if (rightscanval>leftscanval && rightscanval>centerscanval){
    choice = 'r';
  }
  else{
    choice = 'f';
  }
  return choice;
}



void loop(){
   if (irrecv.decode(&results)){ //Check if the remote control is sending a signal
    if(results.value==0xFF6897){ //If an '1' is received, turn on robot

      power=1; 
      enableMotors();
    }
    if(results.value==0xFF4AB5){ //If a '0' is received, turn off robot
      // Serial.println ("Stopping robot");
      stopmove();
      power=0; 
      disableMotors();
    }
    irrecv.resume(); // receive the next value
  }
  
  if(power==1){
  go();  // if nothing is wrong go forward using go() function above.
 
  distance = watch(); // use the watch() function to see if anything is ahead (when the robot is just moving forward and not looking around it will test the distance in front)
  if (distance<distancelimit){ // The robot will just stop if it is completely sure there's an obstacle ahead (must test 25 times) (needed to ignore ultrasonic sensor's false signals)
    
    stopmove(); // Since something is ahead, stop moving.
    turndirection = decide(); //Decide which direction to turn.
    switch (turndirection){
      case 'l':
        turnleft(turntime);
        
        break;
      case 'r':
        turnright(turntime);
        
        break;
      case 'f':
        ; //Do not turn if there was actually nothing ahead
        break;
    }
    thereis=0;
  }
 }  
}

