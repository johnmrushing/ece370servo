#define BACKWARD 5
#define FORWARD 9

#define OUTERIR 11
#define INNERIR 10

int t1, t2;
float thetaDesired = 0, thetaCurrent, thetaChange, thetaError;
int tickCount;
int outerIRRead, innerIRRead;

float kp = 4; // allows for an error of about 10 degrees maximum

char bytes[6];

void setVelocity(float velocity)
{
  /*
   * Updates velocity 0-127-254. Old, proven code from previous homework.
   */
  
  if (velocity >= 127) {
    analogWrite(BACKWARD, 0);
    analogWrite(FORWARD, (velocity - 127) * 2);
  }
  if (velocity < 127) {
    analogWrite(FORWARD, 0);
    analogWrite(BACKWARD, (127 - velocity) * 2);
  }
}

float getAngle(int count)
{
  /*
   * converts from number of ticks observed to a corresponding angle
   */
  
  return count * (90 / 75.8); // 1.18 degrees per tick
}

float PIDControl(float desired, float theta)
{
  /*
   * returns proportional motor control based on error observed
   * between angle desired and current angle
   */
  
  return kp * (desired - theta);
}

void tickInterrupt()
{
  /*
   * reads each IR sensor and compares them
   * based on my quadrature encoder:
   *   SAME = forward
   *   DIFF = backward
   */

  outerIRRead = digitalRead(OUTERIR);
  innerIRRead = digitalRead(INNERIR);
  
  if(outerIRRead != innerIRRead)
    tickCount++; // forward
  else
    tickCount--; // backward
}

void timeUpdate()
{
  /*
   * Allows us to update our motor control value every 20 milliseconds (50 Hz)
   * This is done so we 1) don't update uselessly and
   * 2) so we still update even when the wheel is moving slowly
   */
  
  t1 = millis();
  
  if(t1 - t2 >= 20)
  {
    setVelocity(PIDControl(thetaDesired, thetaCurrent) + 127);
    t2 = t1;
  }
}

void setup()
{
  /*
   * Establishes two serial connections
   * Sets up motor pins
   * Sets up IR sensor pins
   * Sets up interrupt
   * Resets current angle to 0
   */
  
  Serial.begin(9600); // arduino IDE
  Serial1.begin(9600); // pi UART
  
  pinMode(FORWARD, OUTPUT); // motor pin
  pinMode(BACKWARD, OUTPUT); // motor pin
  
  pinMode(OUTERIR, INPUT_PULLUP); // IR sensor doing most of the work
  pinMode(INNERIR, INPUT_PULLUP); // IR sensor for comparison
  
  attachInterrupt(digitalPinToInterrupt(OUTERIR), tickInterrupt, CHANGE); // four interrupts per spin
  
  thetaCurrent = 0; // reset so whatever angle is current is 0
  
  t1 = millis(); // start timer
}

void loop()
{
  /* Primarily, this checks for serial data
   * Six bytes from pi UART (Serial1)
   * (0xFF) (0xFF) (direction) (magnitude/256) (magnitude%256) (checksum)
   * 
   * Then, it updates the motor speed if it's time to do it
   * using the PID controller
   */

  while(true)
  {
    
    if(Serial1.available() >= 6)
    {
  
      for ( char i = 0; i < 7; i++ ) // read six bytes
        bytes[i] = Serial1.read();
      
      if ( bytes[0] == char(0xFF) && bytes[1] == char(0xFF) ) // check header
      {
        char checksum = (bytes[2] + bytes[3] + bytes[4])%256; // same checksum logic as the python script
        if ( checksum == bytes[5] ) // if the checksum passes
        {
          thetaDesired = (bytes[3] * 256) + (unsigned char)bytes[4]; // updated: read unsigned so numbers between 127 and 255 work
          if ( 1 == bytes[2] ) 
            thetaDesired *= -1; // I recognize this is opposite of what's intuitive, but I made my encoder backwards
          Serial.println(thetaDesired); // this is mostly for debugging; sends to arduino IDE
        }

        
        else // bad checksum
          Serial.write("Error in checksum.");
      }
      else // bad header
        Serial.write("Error in header.");
      
    }

    /*
     * Spec is to operate +/- 720 deg
     */
  
    if(thetaDesired > 720)
      thetaDesired = 720;
      
    else if(thetaDesired < -720)
      thetaDesired = -720;
      
    thetaCurrent = getAngle(tickCount);
    
    timeUpdate();
  }
}
