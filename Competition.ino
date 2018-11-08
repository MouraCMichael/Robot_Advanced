/*  Final Robot Sockey Competition Program
 *  Authors: Kaelobb Decker, Jesse Holwerda, Joseph Lentine, Corey Moura
 *  Class: EGR107-01
 *  Instructor: Prof. Zuidema
 *  
 *  Plays Robot Sockey on an Arduino based robot.
 */

#include <NewPing.h> //Library for ultrasonic sensors
#include <sensorbar.h> // Libraries for the line tracker array
#include <Wire.h>
#include <Servo.h> //Library for controlloing servo motors

const unsigned long DELIVERY_INTERVAL = 15000; //Milliseconds since the last delivery of balls before we try to make another
const unsigned long FINAL_DELIVERY = 150000; //At this time, try to deliver balls no matter what
unsigned long lastDelivery = 0; //Milliseconds when we last delivered balls

unsigned long lastLinePivot = 0; //When we last pivoted while following the line.
//It should not take more than 20 pivots reach some sort of intersection
const int TOO_MANY_PIVOTS = 20;
//The robot does not follow the line perfectly. If it has been this many milliseconds since the last pivot,
//the robot has likely fallen off of the line and needs to find it again
const int TOO_LONG_SINCE_LINE_PIVOT = 2000; 

//If we've gone this long since avoiding an obstacle we're probably stuck on someting and need to back up
const int TOO_LONG_SINCE_AVOID = 6000;
//When did we last avoid an obstacles?
unsigned long lastAvoided = 0;

const int MAX_DISTANCE = 10; //Maximum distance for sonar sensors (cm)
const float DANGER_DISTANCE = 10; //How close to the wall before we pivot away (cm)

//What state the front gate is in
enum gateStates
{
  UP,
  DOWN,
  FULL_DOWN
} gateState = DOWN;

//What the robot is trying to do
enum states
{
  COLLECTING_BALLS, //Moving to the next location to start scan
  FOLLOWING_LINE, //Follow line to end
  SCORING_GOAL, //End of line, foward until crash sensors trigger
  LAST_CHANCE,
  STOPPED //Doing nothing
} robotState;


int totalPivots = 0; // How many pivots we've made since getting on the line. 
int consecutiveLasers = 0; //How many times in a row the laser break sensor has triggered

int balls = 0; //How many balls we think we have

// DC motor pins
const int enA = 3;  
const int in1 = 13;
const int in2 = 12;
const int enB = 11;
const int in3 = 9;
const int in4 = 10;

// Microservo pin
const int frontGatePin = 7;

//Ultrasonic sensor pins
const int frontLeftTriggerPin = 5;
const int frontLeftEchoPin = 6;
const int frontRightTriggerPin = 17;
const int frontRightEchoPin = 16;

const int frontLaserPin = 2;
const int leftCrashPin = 4;
const int rightCrashPin = 8;

const int IRLeftPin = A0;
const int IRRightPin = A1;
const int IRShortPin = A2;

Servo frontGate;  // servo control object

const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
SensorBar mySensorBar(SX1509_ADDRESS); // Line tracker array object

//Objects for reading ultrasonic sensors
NewPing frontLeftSonar(frontLeftTriggerPin, frontLeftEchoPin, MAX_DISTANCE);
NewPing frontRightSonar(frontRightTriggerPin, frontRightEchoPin, MAX_DISTANCE);

//Make the motors go as fast as possible
void motorSuperSpeed()
{
  analogWrite(enA, 255);
  analogWrite(enB, 255);
}

//Make the motors go at their normal speed for driving around
void motorFullSpeed() {
  analogWrite(enA, 128); //Controls a PWM signal on pin 9
  analogWrite(enB, 128);
}

//Make the robot go forward for at least delayT milliseconds
void moveForward(int delayT = 0) {
  //Set direction "Forward"
  //Serial.println("Forward");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(delayT);
}

//Make the robot go straight backward for at least delayT milliseconds
void moveBackwardStraight(int delayT = 0) {
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(delayT);
}

//Make the robot pivot right for at least delayT milliseconds
void pivotRight(int delayT = 0) {
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH); 

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(delayT);
}

//Make the robot pivot left for at least delayT milliseconds
void pivotLeft(int delayT = 0) {
  //Pivot "Left"
  //Serial.println("Pivot Left");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(delayT); //wait a sec
}

//Stop both of the motors
void motorStop() {
  //STOP Motor
  analogWrite(enA, 0); //Speed to zero
  analogWrite(enB, 0);
}

//Put the front gate of the robot up. Delay for del seconds to make sure it's up
//Does nothing if the gate is already up
void frontGateUp(int del = 200)
{
  if (gateState != UP)
  {
    frontGate.write(160);
    delay(del);
    gateState = UP;
  }
}

//Put the front gate of the robot down enough to hold balls but avoid obstacles. Delay for del seconds to make sure it's down
//Does nothing if the gate is already down
void frontGateDown(int del = 200)
{
  if (gateState != DOWN)
  {
    frontGate.write(45);
    delay(del);
    gateState = DOWN;
  }
}

//Put the front gate of the robot all the way down. Delay for del seconds to make sure it's down
//Does nothing if the gate is already fully
void frontGateFullDown(int del = 200)
{
  if (gateState != FULL_DOWN)
  {
    frontGate.write(0);
    delay(del);
    gateState = FULL_DOWN;
  }
}

//Read the distance in centimers from an ultrasonic sensor using NewPing
float readSonar(NewPing* sensor)
{
  float med = sensor->ping();
  if (med == 0) med = 999999;
  return med / US_ROUNDTRIP_CM;
}

//Read 3 distances in centimeters and take the median
float readSonarMedian(NewPing* sensor)
{
  float med = sensor->ping_median(3);
  float answer = med / US_ROUNDTRIP_CM; //Constant defined in NewPing.h
  if (answer > 20) answer = 0; //Over 20 indicates a junk reading
  return answer;
}

float IRDistance(int pin)
{
  int reading = analogRead(pin);
  float cm;
  if (pin == IRShortPin)
  {
    cm = 4.8159 * pow(map(reading, 0, 1023, 0, 5000) / 1000.0, -1.2423);
  }
  else
  {
    //Formula is from https://github.com/guillaume-rico/SharpIR/blob/master/SharpIR.cpp
    cm = 27.728 * pow(map(reading, 0, 1023, 0, 5000) / 1000.0, -1.2045);
  }
  return cm;
}

//Take an average of 25 readings using an IR sensor given its pin
float IRDistanceAvg(int pin)
{
  int numTo = 25;
  float avgDist = 0;
  int i;
  for (i = 0; i < numTo; i++)
  {
    avgDist += IRDistance(pin);
    //delayMicroseconds(1);
  }

  avgDist /= numTo;
  return avgDist;
}

//Is the laser currently broken? 1 if yes, 0 if no
int laserBroken()
{
  return digitalRead(frontLaserPin) == HIGH ? 0 : 1;
}

//Is the crash sensor on the pin triggered? 1 if yes, 0 if no
int crashOn(int pin)
{
  return digitalRead(pin) == HIGH ? 0 : 1;
}


void setup() {
  Serial.begin(9600);

  //Initialize pin modes
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(frontLaserPin, INPUT_PULLUP);
  pinMode(leftCrashPin, INPUT);
  pinMode(rightCrashPin, INPUT);

  //Initialize sensor bar
  mySensorBar.setBarStrobe();
  mySensorBar.clearInvertBits();
  mySensorBar.begin();

  //Initialize 
  frontGate.attach(frontGatePin);

  frontGateUp(); //Put the gate up
  motorFullSpeed(); //Normal speed
  pivotLeft(275); //Immediately go into the quadrant to the left
  moveForward(200);

  robotState = COLLECTING_BALLS; //Start off collecting balls
}

/*******   LOGIC FOR STATES BEGINS HERE   *******/

//The main state of the robot. Drive around collecting balls and avoiding obstacles
//25 seconds since the last delivery of balls, try to get onto a line to follow to the goal
void collectingBalls()
{
  moveForward(); 
  if (avoidObstacles()) //Avoid all obstacles. If we avoided an obstacle...
  {
    lastAvoided = millis(); //...record when.
  }
  else if (millis() > lastAvoided + TOO_LONG_SINCE_AVOID) //If we haven't avoided an obstacle in a while, we may be stuck on something
  {
    frontGateDown(); //Put the gate down
    moveBackwardStraight(1000); //Back up and pivot
    pivotLeft(400);
    moveForward(); //Put the gate back up and go forward again
    frontGateUp();
    lastAvoided = millis();
  }
  //If its time to make a delivery - 25 seconds since last and we think we have a ball or it's time for the final delivery
  if ((millis() > lastDelivery + DELIVERY_INTERVAL && balls >= 0) || millis() > FINAL_DELIVERY) 
  { 
    uint8_t rawValue = mySensorBar.getRaw(); //Read the line tracker
    if ((rawValue & 0b00010000) && (rawValue & 0b00001000)) // Middle two sensors triggered
    {
      while ((mySensorBar.getRaw() & 0b00010000) && (mySensorBar.getRaw() & 0b00001000)); //Wait until middle 2 are off line
      frontGateDown(); //Put the gate down and back up onto the line
      moveBackwardStraight(250);
      motorFullSpeed();
      unsigned long startMillis = millis();
      //Pivot until the middle two are back on the line. But if it takes more than a second, 
      //we're probably off of the line and need to try again
      while (!(abs(mySensorBar.getPosition()) > 20 && mySensorBar.getDensity() > 2))
      {
        if(millis() > startMillis + 1000)
        {
          break;
        }
        pivotRight();
      }
      if (millis() <= startMillis + 1000) //If it took less than a second, we're good to begin following the line
      {
        lastLinePivot = millis();
        robotState = FOLLOWING_LINE;
      }
    }   
  }
}

void followingLine()
{
  motorFullSpeed();
  moveForward();
  int density = mySensorBar.getDensity(); //How many of the 8 trackers are reading the line?
  if (density > 5) //If more than 5, we're at an intersection.
  {
    totalPivots = 0;
    robotState = SCORING_GOAL;
  }
  // If we haven't pivoted in a while, we've probably been knocked off of the line
  else if (millis() > lastLinePivot + TOO_LONG_SINCE_LINE_PIVOT) 
  {
    moveBackwardStraight(500);
    robotState = COLLECTING_BALLS;
  }
  //If we've pivoted too many times, we're probably stuck somewhere
  else if (totalPivots == TOO_MANY_PIVOTS) 
  {
    totalPivots = 0;
    moveBackwardStraight(1000); //So get out of there
    pivotLeft(500);
    robotState = COLLECTING_BALLS;
  }
  else if(readSonar(&frontLeftSonar) < MAX_DISTANCE || readSonar(&frontRightSonar) < MAX_DISTANCE)
  {
    totalPivots = 0;
    robotState = SCORING_GOAL;
  }
  else
  {
    int pos = mySensorBar.getPosition();
    if (pos < -40) //Need to pivot left
    {
      while (mySensorBar.getPosition() < -40)
      {
        pivotLeft();
        lastLinePivot = millis();
      }
      totalPivots++;
    }
    else if (pos > 40) //Need to pivot right
    {
      while (mySensorBar.getPosition() > 40)
      {
        pivotRight();
        lastLinePivot = millis();
      }
      totalPivots++;
    }
    else //No need to pivot
    {
      moveForward();
    }
  }
}

// Shoot the balls out of the robot into a goal
void scoringDance()
{
  motorFullSpeed();
  moveBackwardStraight(200); //Back up
  motorStop();
  frontGateUp(500); //Open the gate while stopped
  motorFullSpeed();
  moveBackwardStraight(400);
  frontGateDown();
  moveForward(700); //Give the balls a shove
  moveBackwardStraight(1000); //Go somewhere else
  pivotLeft(600);
}

// The robot has balls and is at an intersection, possibly the goal. If so, try to score them
void scoringGoal()
{
  motorFullSpeed();
  moveForward(150); //Go forward a little bit
  //If we were at the goal, we should now be off the line
  if(mySensorBar.getDensity() == 0 || (readSonar(&frontLeftSonar) < MAX_DISTANCE || readSonar(&frontRightSonar) < MAX_DISTANCE) )
  {
    scoringDance(); //So score the balls
    balls = 0; //Reset the ball counter
    robotState = COLLECTING_BALLS; //Go back to collectin
    lastDelivery = millis();
    lastAvoided = millis();
  }
  else //If we're still on the line, we just cross the intersection at the middle
  {
    lastLinePivot = millis();
    totalPivots = 0;
    robotState = FOLLOWING_LINE; //So keep following the line
  }
}

//Only 5 seconds left. Drive forward and hope something good happens/
//Joe wanted his name here, so here it is.
void lastChance() 
{
  frontGateUp();
  motorSuperSpeed();
  moveForward();
}

//All code for avoiding obstacles and walls using the sensors
//Returns 1 if it avoided something, 0 otherwise
int avoidObstacles()
{
  int avoided = 1;

  //Read all of the sensors
  float leftS = readSonar(&frontLeftSonar);
  float rightS = readSonar(&frontRightSonar);
  //float frontIR = IRDistanceAvg(IRShortPin);
  float leftDist = IRDistanceAvg(IRLeftPin);
  float rightDist = IRDistanceAvg(IRRightPin);
  int leftCrash = crashOn(leftCrashPin);
  int rightCrash = crashOn(rightCrashPin);
  int laserBroke = laserBroken();
  int density = mySensorBar.getDensity();
  int pos = mySensorBar.getPosition();

  if (laserBroke) //If something broke the front laser
  {
    consecutiveLasers++; //Increment laser counter
    Serial.println("Laser");
    frontGateDown();
    //Always back up a little bit as objects may move past the laser
    moveBackwardStraight(200);
    unsigned long backwardMillis = millis();
    //Back up until the laser is no longer broken and record how long it takes.
    while (laserBroken() && millis() < backwardMillis + 1300)
    {
      moveBackwardStraight();
    }
    //If the laser is still broken, a ball has likely gotten stuck under the gate
    if (millis() >= backwardMillis + 1300)
    {
      motorStop(); //Hold still for a second
      delay(1000);
      //Put the gate up to release the ball, drive forward a little to collect it, then keep going
      frontGateUp();
      moveForward(200);
      frontGateDown();
      motorFullSpeed();
    }
    //If it took less than 100 millis to unbreak the laser, it may be a ball
    if (millis() < backwardMillis + 100 && consecutiveLasers < 2)
    {
      balls++;
      frontGateUp();
    }
    //If it took more than 100 millis or the laser keeps breaking, we're likely at
    //an obstacle. Pivot away in a random direction.
    else if (millis() >= backwardMillis + 100 || consecutiveLasers > 1)
    {
      consecutiveLasers = 0;
      int dir = random(1);
      int len = random(301) + 100;
      if (dir == 0)
      {
        pivotRight(len);
      }
      else
      {
        pivotLeft(len);
      }
    }
    motorFullSpeed();
    moveForward();
  }
  //Front IR sensor detecting flow
  /*
  else if (frontIR > 8)
  {
    moveBackwardStraight(700);
    pivotRight(400);
    moveForward();
  }
  */
  //Either ultrasonic detects top of goal
  else if(leftS < MAX_DISTANCE)
  {
    consecutiveLasers = 0;
    frontGateDown();
    motorSuperSpeed();
    moveBackwardStraight(850);
    motorFullSpeed();
    pivotRight(400);
    moveForward();
  }
  else if(rightS < MAX_DISTANCE)
  {
    consecutiveLasers = 0;
    frontGateDown();
    motorSuperSpeed();
    moveBackwardStraight(850);
    motorFullSpeed();
    pivotLeft(400);
    moveForward();
  }
  //Both of the top IRs report we're near a wall
  else if (leftDist < DANGER_DISTANCE && rightDist < DANGER_DISTANCE)
  {
    consecutiveLasers = 0;
    Serial.println("Left and right IR");
    frontGateDown();
    //If the line tracker is also picking up the line, we're near a goal.
    if(density > 0)
    {
      //Back up away from it quickly
      unsigned long bMill = millis();
      while(mySensorBar.getDensity() != 0 && millis() < bMill + 1000)
      {
        motorSuperSpeed();
        moveBackwardStraight();
      }
      motorFullSpeed();
    }
    //If not, we're just at a wall. Back up a little and pivot away
    else
    {
      moveBackwardStraight(200);
    }
    //We can't really tell if we need to pivot left or right, so just pick at random.
    int dir = random(1);
    int len = 800 + random(201);
    if (dir == 0)
    {
      pivotRight(len);
    }
    else
    {
      pivotLeft(len);
    }
    motorFullSpeed();
    moveForward();
  }
  //Only the left IR is detecting a wall
  else if (leftDist < DANGER_DISTANCE)
  {
    consecutiveLasers = 0;
    Serial.println("Left IR");
    frontGateDown();
    //Same logic as above
    if(density > 0)
    {
      unsigned long bMill = millis();
      while(mySensorBar.getDensity() != 0 && millis() < bMill + 1000)
      {
        motorSuperSpeed();
        moveBackwardStraight();
      }
      motorFullSpeed();
    }
    else
    {
      moveBackwardStraight(200);
    }
    pivotRight(300 + random(201));
    motorFullSpeed();
    moveForward();
  }
  //Only the right sensor is detecting a wall
  else if (rightDist < DANGER_DISTANCE)
  {
    consecutiveLasers = 0;
    Serial.println("Right IR");
    frontGateDown();
    //Same logic as above
    if(density > 0)
    {
      unsigned long bMill = millis();
      while(mySensorBar.getDensity() != 0 && millis() < bMill + 1000)
      {
        motorSuperSpeed();
        moveBackwardStraight();
      }
      motorFullSpeed();
    }
    else
    {
      moveBackwardStraight(200);
    }
    pivotLeft(300 + random(201));
    motorFullSpeed();
    moveForward();

  }
  //Left crash sensor has bumped something
  else if (leftCrash)
  {
    consecutiveLasers = 0;
    Serial.println("Left crash");
    frontGateDown();
    //Back up until it's no longer triggers and pivot a little to the right
    while (crashOn(leftCrashPin))
    {
      moveBackwardStraight();
    }
    pivotRight(200);
    motorFullSpeed();
    moveForward();
  }
  //Right crash sensor has bumped something
  else if (rightCrash)
  {
    consecutiveLasers = 0;
    Serial.println("Right crash");
    frontGateDown();
    //Same logic as above
    while (crashOn(rightCrashPin))
    {
      moveBackwardStraight();
    }
    pivotLeft(200);
    motorFullSpeed();
    moveForward();
  }
  /*
  //We are pointing straight out of a goal
  else if (density > 6 && abs(pos) < 20)
  {
    //So don't drive out
    consecutiveLasers = 0;
    frontGateDown();
    moveBackwardStraight(400);
    pivotLeft(400);
    moveForward();
  }
  */
  //We can't see any obstacles
  else
  {
    Serial.println("Nothing");
    avoided = 0;
    //Don't put the gate back up until we've gone at least 500 millis without detecting an obstacle
    if (millis() > lastAvoided + 500)
    {
      frontGateUp();
    }
  }
  return avoided;
}

void loop()
{
  //Stop at 3:02
  if (millis() > 182000)
  {
    robotState = STOPPED;
  }
  //Only 5 seconds left, just drive forward
  else if(millis() > 175000)
  {
    robotState = LAST_CHANCE;
  }
  //Otherwise call the appropriate function for the robot's current state.
  switch (robotState)
  {
    case COLLECTING_BALLS:
      collectingBalls();
      break;
    case FOLLOWING_LINE:
      followingLine();
      break;
    case SCORING_GOAL:
      scoringGoal();
      break;
    case LAST_CHANCE:
      lastChance();
      break;
    default:
      motorStop();
      break;
  }
}
