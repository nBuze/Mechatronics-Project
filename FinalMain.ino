///////////////////////////////
// Elec 299 - Winter 2017
///////////////////////////////
// FinalProjectCode
// 
// Group Members:
// Nolan Buzanis 
// Miguel Martins 
// Nick Guida 


#include <Servo.h>   //Include the library related with servo-controlling function
#include <QSerial.h> //Include the library related to the IR communication

//IR Bug-Eye 
#define IRCOMM 3
#define IRCOMM2 3
QSerial IRserial; //First IR sensor
QSerial IRserial2; //Second IR sensor for increased accuracy

#define GRIP_PIN 11 //For the grip (ball sensor)

//Servo Pins
#define PAN 8
#define TILT 9
#define GRIP 10

//Initial servo positioning
Servo pan_servo, tilt_servo, grip_servo;
int pan_pos = 90, tilt_pos = 160, grip_pos = 40; 

//LED pins
#define LED 13

//Motor Pins
#define RD 7
#define LD 4
#define RS 6
#define LS 5

//Pushbutton Pin
#define BUTTON 2

//Analog Pins for linetracker function
#define LTL A1  //Linetracker Left
#define LTC A2  //Linetracker Centre
#define LTR A0  //Linetracker Right

//Linetracking Threshold
#define LTHRES 800
#define CTHRES 800
#define RTHRES 800    //Must be adjusted according to your own sensor
#define driveSpeed 100

byte Switch = 0; //Flag variable used to control the linetracking function
#define Delta 20 //Change in speed when robot is off the line
int Rotate = 0; //Controls the head rotation timing at beginning of main loop

//Pivot Constants
#define turnSpeed 95
#define LEFT (-1)
#define RIGHT 1
#define PIVOT90 550
#define PIVOT180 1075

void setup() {
  //Initial pin setups
  pinMode(RD,OUTPUT);
  pinMode(LD,OUTPUT);
  pinMode(RS,OUTPUT);
  pinMode(LS,OUTPUT);
  pinMode(BUTTON,INPUT);
  pinMode(LTL, INPUT);
  pinMode(LTC, INPUT);
  pinMode(LTR, INPUT);
  pinMode(GRIP_PIN,INPUT);
  pinMode(LED,OUTPUT);
  pinMode(IRCOMM,INPUT);
  
  //IR Communication
  IRserial.attach(IRCOMM,-1);
  IRserial2.attach(IRCOMM2,-1);

  //Attach Servo to Pins
  pan_servo.attach(PAN);
  tilt_servo.attach(TILT);
  grip_servo.attach(GRIP);
  
  //Set default servo positions
  pan_servo.write(pan_pos);
  //tilt_servo.write(tilt_pos);
  grip_servo.write(grip_pos);
  
  //Turn off LEDs
  digitalWrite(LED,LOW);

  pushButton(); //Push button to start the program
  
}

void loop() {
  //Turn LED on when the robot is ready to recieve an IR transmission
digitalWrite(LED, HIGH);

while(true){

//Head of the robot spins to sense IR transmission at different points
if(Rotate == 0){
  setPan(0);
  }
  if(Rotate == 10){
  setPan(90);
  }
  if(Rotate == 20){
  setPan(180);
  }
  
Rotate += 1; //Increment rotate counter by 1 each iteration

  char val = IRserial.receive(200);
  char val2 = IRserial2.receive(200);
  
// 1 is read from IR
if (val == '1' || int(val) == 49 || val2 == '1' || int(val2) == 49){\
  digitalWrite(LED, LOW);

  // Set the gripper to look up so it is out of the way
  setPan(90);
  setTilt(160);

  // Initiate lineFollower function until it reaches the black 'T'
  Switch = 0;
  while(Switch == 0){
  followLine();
   }
  Backup(); //Backup from wall so gripper has room to come down
  setTilt(65);
  Forward();
  closeGrip(); //Grip the ball and exit this function ONLY when the ball is sensed
  delay(200);
  setTilt(160);
  Backup(); // Backs up so the robot has room to pivot
  Pivot(RIGHT, PIVOT90); //Pivots until the robot senses the first black line (really a 180 degrees turn)
  Stop();

  // Call the lineFollower twice since the first function will end at the crossroads
  Switch = 0;
  while(Switch == 0){
  followLine();
   }
   Forward();
   // Back at center now
   Switch = 0;
  while(Switch == 0){
  followLine();
   }

  // Dump the ball in the goal
  setTilt(120);
  setGrip(50); //Loose gripper
  setTilt(160);
  Backup();
  Backup();
  Pivot(RIGHT, PIVOT90); //Pivots 180 degrees until it hits a black line

  // Send robot to the center again
  Switch = 0;
  while(Switch == 0){
  followLine();
   }
   setTilt(80);
   Forward(); //Make the robot move forward a few cm so it is centered better for the next move
   Forward();
   Forward();
  }

// Many parts from '2' and '0' are identical to '1' so refer to '1' for commenting
// 2 is read
else if (val == '2' || int(val) == 50 || val2 == '2' || int(val2) == 50){
  digitalWrite(LED, LOW);
  setPan(90);
  setTilt(160);
  Pivot(LEFT, PIVOT90);
  Switch = 0;
  while(Switch == 0){
  followLine();
   }
  Backup();
  setTilt(65);
  Forward();
  closeGrip();
  delay(200);
  setTilt(160);
  Backup();
  Pivot(RIGHT, PIVOT90);
  Stop();
  Switch = 0;
  while(Switch == 0){
  followLine();
   }
   Forward();
   // Back at center now
   Pivot(RIGHT, PIVOT90);
   Switch = 0;
  while(Switch == 0){
  followLine();
   }
   //Backup();
  setTilt(120);
  setGrip(50); //Loose gripper
  setTilt(160);
  Backup();
  Backup();
  Pivot(RIGHT, PIVOT90);
  Switch = 0;
  while(Switch == 0){
  followLine();
   }
  setTilt(80);
  Forward();
  Forward();
  Forward();
  }
  
// 0 is read
else if (val == '0' || int(val) == 48 || val2 == '0' || int(val2) == 48){
  digitalWrite(LED, LOW);
  setPan(90);
  setTilt(160);
  Pivot(RIGHT, PIVOT90);
  Switch = 0;
  while(Switch == 0){
  followLine();
   }
  Backup();
  setTilt(65);
  Forward();
  closeGrip();
  setTilt(160);
  Backup();
  Pivot(RIGHT, PIVOT90);
  Stop();
  Switch = 0;
  while(Switch == 0){
  followLine();
   }
   Forward();
   // Back at center now
   Pivot(LEFT, PIVOT90);
   Switch = 0;
  while(Switch == 0){
  followLine();
   }
   //Backup();
  setTilt(120);
  setGrip(50); //Loose gripper
  setTilt(160);
  Backup();
  Backup();
  Pivot(RIGHT, PIVOT90);
  Switch = 0;
  while(Switch == 0){
  followLine();
   }
  setTilt(80);
  Forward();
  Forward();
  Forward();
  }

delay(200);
if(Rotate == 29){
  Rotate = 0;
  }
}
}

// Wait until button is pressed and then released
void pushButton(){
  while(digitalRead(BUTTON)){}
  delay(50);
  while(!digitalRead(BUTTON)){}
  }

// lineFollower function to follow the black line in the arena
void followLine(){
  // Read sensor values
  int leftVal = analogRead(LTL);
  int centreVal = analogRead(LTC);
  int rightVal = analogRead(LTR);
  // Set the wheels to drive in a forward direction
  digitalWrite(RD, HIGH);
  digitalWrite(LD, HIGH);
  delay(50);

  //Line centred, drive straight
 if(leftVal<LTHRES && centreVal>CTHRES && rightVal<RTHRES){
 analogWrite(RS,driveSpeed);
 analogWrite(LS,driveSpeed);  //Please to use the lowest speed that can make the robot move!!!
 }

 //Hits a solid line
 else if(leftVal > LTHRES && centreVal > CTHRES && rightVal > RTHRES){
    Switch = 1;
    Stop();
  }
 
 //Veering right, move power to right motor
 else if(leftVal>LTHRES && centreVal<CTHRES && rightVal<RTHRES){
 analogWrite(RS,driveSpeed+Delta); //Increase speed of right wheel and decrease speed of left wheel
 analogWrite(LS,driveSpeed-Delta);
 }
 
 else if(leftVal>LTHRES && centreVal>CTHRES && rightVal<RTHRES){
 analogWrite(RS,driveSpeed+Delta);
 analogWrite(LS,driveSpeed-Delta);
 }
 
 //Veering left, move power to left motor
 else if(leftVal<LTHRES && centreVal<CTHRES && rightVal>RTHRES){
 analogWrite(RS,driveSpeed-Delta);
 analogWrite(LS,driveSpeed+Delta);
 }
 
 else if(leftVal<LTHRES && centreVal>CTHRES && rightVal>RTHRES){
 analogWrite(RS,driveSpeed-Delta);
 analogWrite(LS,driveSpeed+Delta);
 }

 else{ //If nothing else, drive straight
  analogWrite(RS,driveSpeed);
  analogWrite(LS,driveSpeed);
  }
 
}

  // Pivot function makes the robot turn until it senses a black line
 void Pivot(int whichWay, int Angle)
{ 
  // Read sensor values
  int leftVal = analogRead(LTL);
  int centreVal = analogRead(LTC);
  int rightVal = analogRead(LTR);
  
    if( whichWay == LEFT )
    {//Pivot LEFT
      //Motors to drive forward
      digitalWrite(LD,LOW);   
      digitalWrite(RD,HIGH);
    }

    else if( whichWay == RIGHT )
    {//Pivot RIGHT 
      //Motors to drive forward
      digitalWrite(LD,HIGH);   
      digitalWrite(RD,LOW);
      } 
      
    //robot initializes its turn so the sensors are off the black line
     analogWrite(LS,turnSpeed);
     analogWrite(RS,turnSpeed); 
     delay(850);
     
      while(true){
      // Set motors to drive a certain predefined speed
     analogWrite(LS,turnSpeed);
     analogWrite(RS,turnSpeed);
     // Read sensor values
     leftVal = analogRead(LTL);
    centreVal = analogRead(LTC);
    rightVal = analogRead(LTR);
     delay(20); 
     // If any sensor senses a black line, break out of this function
     if(centreVal > CTHRES || rightVal > RTHRES || leftVal > LTHRES){
     break;
      }  
    }

    // Robot will stop its turn after noticing a second black line
    if(Angle == PIVOT180){
      analogWrite(LS,turnSpeed);
     analogWrite(RS,turnSpeed); 
     delay(600);
     
      while(true){
       //Motor to drive a speed xxx
     analogWrite(LS,turnSpeed);
     analogWrite(RS,turnSpeed);
     leftVal = analogRead(LTL);
    centreVal = analogRead(LTC);
    rightVal = analogRead(LTR);
     delay(20); 
     if(leftVal > LTHRES || centreVal > CTHRES || rightVal > RTHRES){
     break;
      }  
    }
      
      }
    
  analogWrite(LS,turnSpeed);
     analogWrite(RS,turnSpeed); 
     delay(100);
    
      Stop(); // Make the robot stop
return;    
}

void Stop(){
    analogWrite(LS,0); //Set left and right motor speeds to zero
    analogWrite(RS,0);
  }

void Backup()
{
  //Motors to drive backward
  digitalWrite(LD,LOW);   
  digitalWrite(RD,LOW);  
  
  //Motor to drive a speed xxx
  analogWrite(LS,driveSpeed);
  analogWrite(RS,driveSpeed);
  delay(300); //Backup for a specified ammount of time
  Stop();
  return;
}

void Forward()
{
  //Motors to drive forward
  digitalWrite(LD,HIGH);   
  digitalWrite(RD,HIGH);  
  
  //Motor to drive a speed xxx
  analogWrite(LS,driveSpeed);
  analogWrite(RS,driveSpeed);
  delay(300); //Go forward for a specified ammount of time
  Stop();
  return;
}

// Close the gripper on the robot until a ball is pressed
void closeGrip(){

  while( digitalRead(GRIP_PIN) == LOW ) //When nothing is detected by the pressure sensor
    {
      grip_pos += 1;
      grip_servo.write(grip_pos);
      delay(50);

      if(digitalRead(GRIP_PIN) == HIGH){
        grip_pos += 3;
        grip_servo.write(grip_pos);
        break;
      }
    //To protect the servo, if nothing is grabed by the gripper, stop closing
      if( grip_pos > 110 ){
        setGrip(50);
        // The gripper did not pickup the ball so try again:
        closeGrip();
      }
    }
    
  //
    if( grip_pos < 110 ) 
    {
        digitalWrite(LED,HIGH);  //set the indicator
        // If ball is gripped, turn the LED on
    }
    else
    {
      setGrip(50);
    }
  }

  //The purpose of these is to make the servo move smoothly and slowly
void setPan(int pos) {
 
  if( pos > pan_pos )
  {//Set Pan Servo position if pos is greater than current pan_pos
    for(pan_pos; pan_pos <= pos; pan_pos += 1) 
    {  
      pan_servo.write(pan_pos);
      delay(5);
    }
  }else //if( pos < pan_pos )
  {//Set Pan Servo position if pos is smaller than current pan_pos
    for(pan_pos; pan_pos >= pos; pan_pos -= 1) 
    {
      pan_servo.write(pan_pos);
      delay(5);
    }
  }
  return;  
}// end setPan

void setTilt(int pos) {
  
  //Limit the minimum tilt position possible
  //TO AVOID HITTING THE TOP PLATE
  if(pos < 20) {
    pos = 20;
  }// end if

  if( pos > tilt_pos )
  {//Set Tilt Servo position if pos is greater than current tilt_pos
    for(tilt_pos; tilt_pos <= pos; tilt_pos += 1) 
    {
      tilt_servo.write(tilt_pos);
      delay(5);    
    }
  }else //if (pos < tilt_pos)
  {//Set Tilt Servo position if pos is smaller than current tilt_pos
    for(tilt_pos; tilt_pos >= pos; tilt_pos -= 1) 
    {
      tilt_servo.write(tilt_pos);
      delay(5);    
    }
  }
  return;  
}// end setTilt

void setGrip(int pos) {

  if( pos > grip_pos )
  {//Set Grip Servo position if pos is greater than current grip_pos 
    for(grip_pos; grip_pos <= pos; grip_pos += 1) 
    {
      grip_servo.write(grip_pos);
      delay(5);
    }    
  } else //if (pos > tilt_pos)
  {//Set Grip Servo position if pos is smaller than current grip_pos
    for(grip_pos; grip_pos >= pos; grip_pos -= 1) 
    {
      grip_servo.write(grip_pos);
      delay(5);
    } 
  }
 
  return;
}// end setGrip


