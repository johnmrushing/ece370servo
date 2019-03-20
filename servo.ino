#define forward 5
#define backward 9
#define outerIR 11
#define innerIR 10

const int numReadings = 10; //how long our running average goes
int readIndex = 0; //running average index
float readings[numReadings], total, average; //variables for running averages
//float requested; //presently used only for debugging; represents velocity control

int ticks, angleToTicks; //v2; ticks is just a count of total interrupts; 4 per motor rotation

float currentAngle, desiredAngle, angleChange;

boolean endOfRotation = false; //v2; enable braking

int sign; //direction
int detectIn; //what the inner IR sensor has *right now*
int detectIn_old; //what the inner IR sensor had previously
int detectOut; //what the outer IR sensor has *right now*
int detectOut_old; //what the outer IR sensor had previously
int i = 0;
//float currentOffset = 1, currentVelocity = 127; //starting values for velocity; debugging

float t1 = 0, t2 = 1, tDelta, RPSinput = 0, RPSoutput; //times, hertz

float thetaDesired = 0; //desired angle
float thetaNow = 0; //actual angle
float error = 0; //desired angle - actual angle
float adjustment = 0; //"output"

float Kp = 0; //our "current" kp value; changes based on error value

//the following are tweakable to get appropriate reponses
float Kp4 = 0.5; //almost done
float Kp3 = 0.9;
float Kp2 = 1.7;
float Kp1 = 3.0; //just starting

void setVelocity(float velocity) {
  if (velocity >= 127) {
    analogWrite(backward, 0);
    analogWrite(forward, (velocity - 127) * 2);
  }
  if (velocity < 127) {
    analogWrite(forward, 0);
    analogWrite(backward, (127 - velocity) * 2);
  }
  //Serial.println(velocity);
}

void lockMotor(){
  if(endOfRotation){
    
    detachInterrupt(digitalPinToInterrupt(outerIR));
    
    analogWrite(forward, 0); // lock motor
    analogWrite(backward, 0);
    
    attachInterrupt(digitalPinToInterrupt(outerIR), getTick, CHANGE);
  }
}

void goToAngle(float angle){

  if(angle > 720)
  {                                           //check if input degrees is greater than +- 720
    angle = 720;
  }
  
  else if(angle < -720)
  {
    angle = -720;
  }
  
  angleChange = angle - currentAngle;
  
  angleToTicks = abs(angleChange / 360.0) * 75.8 * 4; //75.8 gearbox ratio; 4 ticks per input rotation

  endOfRotation = false;
  
  if(angleChange > 0)
  {
    setVelocity(254);
  }
  
  else if(angleChange < 0)
  {
    setVelocity(0);
  }
  
  else //no angle change; no action
  {
    
  }
}

void getTick(){

  /*
   * This function is a hack; it attempts to overcorrect for inertia, since I'm unable to use a kp appropriately
   * It finds when we're relatively close to where we need to be and locks the motor
   */

  ticks++;
  
  if(angleChange <= -540) //big negative change
  {
    if(ticks > (angleToTicks - 400) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle;  
      lockMotor();
    }
  }
  
  else if(angleChange <= -360)
  {
    if(ticks > (angleToTicks - 200) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle;  
      lockMotor();
    }
  }
  else if(angleChange <= -270)
  {
    if(ticks > (angleToTicks - 150) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle;  
      lockMotor();
    }
  }
    
  else if(angleChange <= -180)
  {
    if(ticks > (angleToTicks - 110) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle;  
      lockMotor();
    }
  }
    
  else if(angleChange <= -150)
  {
    if(ticks > (angleToTicks - 110) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle;  
      lockMotor();
    }
  }
  
  else if(angleChange <= -135)
  {
    if(ticks > (angleToTicks - 95) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle;  
      lockMotor();
    }
  }
    
  else if(angleChange <= -90)
  {
    if(ticks > (angleToTicks - 60) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle;  
      lockMotor();
    }
  }
    
  else if(angleChange <= -60)
  {
    if(ticks > (angleToTicks - 40) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle;  
      lockMotor();
    }
  }
  else if(angleChange <= -30)
  {
    if(ticks > (angleToTicks - 25) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle;  
      lockMotor();
    }
  }
    
  else if(angleChange <= -15) //small rotation; not a lot of inertia to overcome
  {
    if(ticks > (angleToTicks - 0) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle;  
      lockMotor();
    }
  }
    
  else if(angleChange <= 15) //small rotation; not a lot of inertia to overcome
  {
    if(ticks > (angleToTicks - 0) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle;  
      lockMotor();
    }
  }
    
  else if(angleChange <= 30)
  {
    if(ticks > (angleToTicks - 25) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle;  
      lockMotor();
    }
  }
    
  else if(angleChange <= 60)
  {
    if(ticks > (angleToTicks - 40) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle;  
      lockMotor();
    }
  }
    
  else if(angleChange <= 90)
  {
    if(ticks > (angleToTicks - 60) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle;  
      lockMotor();
    }
  }
  
  else if(angleChange <= 135)
  {
    if(ticks > (angleToTicks - 95) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle;  
      lockMotor();
    }
  }
  else if(angleChange <= 180)
  {
    if(ticks > (angleToTicks - 110) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle;  
      lockMotor();
    }
  }
    
  else if(angleChange <= 270)
  {
    if(ticks > (angleToTicks - 140) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle;  
      lockMotor();
    }
  }
    
  else if(angleChange <= 360)
  {
    if((ticks > (angleToTicks - 150)) && !endOfRotation)
    {
      endOfRotation = true;      
      lockMotor();
    } 
  }
  
  else if(angleChange <= 540)
  {
    if(ticks > (angleToTicks - 200) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle;  
      lockMotor();
    }
  }
  
  else // big positive change
  {
    if((ticks > (angleToTicks - 400)) && !endOfRotation)
    {
      endOfRotation = true;
      currentAngle = desiredAngle; 
      lockMotor();
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  Serial1.begin(9600);
  
  pinMode(forward, OUTPUT);
  pinMode(backward, OUTPUT);

  pinMode(outerIR, INPUT_PULLUP);
  pinMode(innerIR, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(outerIR), getTick, CHANGE);
  
  currentAngle = 0;

  for (int thisReading = 0; thisReading < numReadings; thisReading++) { //reset running average
    readings[thisReading] = 0;
  }
}
void loop() {
  // put your main code here, to run repeatedly:
  
  while(1)
  {
    //if(Serial1.available()) { //read from pi
    if(Serial.available()) { //read from USB
      thetaDesired = Serial.parseFloat();
      Serial.println(thetaDesired);
      //currentOffset = 1;
      ticks = 0;
      goToAngle(thetaDesired);
    }
  }
}

/*
float adjustmentFunc(float thetaDesired, float theta)
{
  error = thetaDesired - theta;

  if (abs(error) >= 360)
  {
    Kp = Kp4;
  }
  else if (abs(error) >= 180)
  {
    Kp = Kp3;  
  }
  else if (abs(error) >= 90)
  {
    Kp = Kp2;
  }
  else
  {
    Kp = Kp1;
  }
}
*/

/*
void parseRPS()
{
  t1 = t2;
  t2 = millis();

  tDelta = t2 - t1;

  total = total - readings[readIndex];
  readings[readIndex] = tDelta;
  total = total + readings[readIndex];
  readIndex++;

  if(readIndex >= numReadings)
    readIndex = 0;

  average = total / numReadings;

  RPSinput = 1000 / (4 * average); // time for one revolution of input
  RPSoutput = RPSinput / 75.81; // time for one revolution of output

  //Serial.println(RPSinput);
  //Serial.println(RPSoutput);
}
*/

/*
void checkRPS()
{
  if (abs(requested - RPSoutput) > 0.005)
  {
    //currentOffset *= 1.01;
    //if(currentOffset > 1.5) currentOffset = 1.5;
    setRPS();
  }
  else
  {
    currentOffset = 1;
  }
}
*/

/*
void setRPS()
{
  if(requested < 0) sign = -1;
  else sign = 1;

  if (requested > RPSoutput)
  {
    //setVelocity(currentVelocity + (currentOffset));
    setVelocity(254);
  }
  else if (requested < RPSoutput)
  {
    //setVelocity(currentVelocity - (currentOffset));
    setVelocity(0);
  }
}
*/
