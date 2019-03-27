#define BACKWARD 5
#define FORWARD 9

#define OUTERIR 11
#define INNERIR 10

int t1, t2;
float thetaDesired = 0, thetaCurrent, thetaChange, thetaError;
int tickCount, ticksDesired;
int outerIRRead, innerIRRead;

float kp = 4; // allows for an error of about 10 degrees maximum

char bytes[6];

void setVelocity(float velocity) {
  if (velocity >= 127) {
    analogWrite(BACKWARD, 0);
    analogWrite(FORWARD, (velocity - 127) * 2);
  }
  if (velocity < 127) {
    analogWrite(FORWARD, 0);
    analogWrite(BACKWARD, (127 - velocity) * 2);
  }
}

float getAngle(int count){
  return count * (90 / 75.8); //1.2 degrees per tick
}

float PIDControl(float desiredtheta, float theta){
  return kp * (desiredtheta - theta);
}

void tickInterrupt(){

  outerIRRead = digitalRead(OUTERIR);
  innerIRRead = digitalRead(INNERIR);
  
  if(outerIRRead != innerIRRead)
    tickCount++;
  else
    tickCount--; 
}

void timeUpdate()
{
  t1 = millis();
  if(t1 - t2 >= 20){
    setVelocity(PIDControl(thetaDesired, thetaCurrent) + 127);
    t2 = t1;
  }
}

void setup() {

  Serial.begin(9600); // arduino IDE
  Serial1.begin(9600); // pi UART
  
  pinMode(FORWARD, OUTPUT);
  pinMode(BACKWARD, OUTPUT);
  
  pinMode(OUTERIR, INPUT_PULLUP);
  pinMode(INNERIR, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(OUTERIR), tickInterrupt, CHANGE);
  
  thetaCurrent = 0; // reset so whatever angle is current is 0
  
  t1 = millis(); // start timer
}

void loop() {

  /*
   * Six bytes from pi UART (Serial1)
   * (0xFF) (0xFF) (direction) (magnitude/256) (magnitude%256) (checksum)
   */

  while(true){
    
    if(Serial1.available() >= 6){
  
      for ( char i = 0; i < 7; i++ )
      {
        bytes[i] = Serial1.read();
      }
      
      if ( bytes[0] == char(0xFF) && bytes[1] == char(0xFF) )
      {
        char checksum = (bytes[2] + bytes[3] + bytes[4])%256;
        if ( checksum == bytes[5] )
        {
          thetaDesired = (bytes[3] * 256) + (unsigned char)bytes[4];
          if ( 1 == bytes[2] )
            thetaDesired *= -1;
          Serial.println(thetaDesired);
        }
        else // bad checksum
        {
          Serial.write("Error in checksum.");
        }
      }
      else // bad header
      {
        Serial.write("Error in header.");
      }
      
    }
  
    if(thetaDesired > 720)
      thetaDesired = 720;
      
    else if(thetaDesired < -720)
      thetaDesired = -720;
      
    thetaCurrent = getAngle(tickCount);
    
    timeUpdate();
  }
}
