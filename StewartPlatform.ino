/* Code for Stewart Platform
 * Arduino Uno
 * 6 Servos
 * Wii Nunchuck
 *
 * Arnaud DESSEIN
 * https://adessein@bitbucket.org/adessein/stewartplatform
 */

/* (yaw, psi)
 *    (Z)----------->  X  (roll, phi)
 *     |
 *     |
 *     |
 *     | 
 *    \|/
 *     Y
 *    
 *     Y (pitch, theta)
 */

#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <WiiChuck.h>
#include <Timer.h>
#include <Adafruit_STMPE610.h>

/* PH is the platform height (in the base framework). It is used for the rest position.

   ZB is the vertical distance between the base framework (bottom) 
   and the servo axes.
   
   ZP is the vertical distance between the platform framework (screen) 
   and the arm axes. It is only used here to make the gerometry
   easier to change
 */

#define maxAnglePlate 15 // in degrees
#define maxAngleServos 40 // in degrees
#define ZB 45e-3 // vertial distance between base origin and servo axes
#define ZP -2e-2 // ertial distance between plaform orign and attachement points
#define H0 15.79e-2 // Home position [m]

#define plateXmin -4e-2
#define plateXmax 4e-2
#define plateYmin -4e-2
#define plateYmax 4e-2
#define plateZstep 1e-3
#define plateZmin 14e-2
#define plateZmax 18e-2

#define joyXmin -103
#define joyXmax 96
#define joyYmin -103
#define joyYmax 98
#define joyPhiMin -70
#define joyPhiMax 70
#define joyThetaMin -70
#define joyThetaMax 70

#define stmpeXmin 140
#define stmpeXmax 3800
#define stmpeYmin 210
#define stmpeYmax 3900
#define stmpeX0 2000
#define stmpeY0 2000

#define fcof 0.4

#define serialPeriod 100 //ms
#define serialSpeed 115200 //bauds

#define modePin 12 // do not use 13 is has a led connected to it

float CP=40, 
      CI=210, 
      CD=1300;
/***************************************************/
/************* Class OneDimControl ******************/
/***************************************************/

class OneDimController
{
    public:
        OneDimController(float p, float i, float d);
        float calCommand(float err);
    private:
        float _p;
        float _i;
        float _d;
        float _error[2];
};
        

OneDimController::OneDimController(float p, float i, float d) : _p(p), _i(i), _d(d)
{
}

float OneDimController::calCommand(float err)
{
    // Shift error register and append new value
    _error[0] = _error[1];
    _error[1] = err;

    // Calculate the command (PID controller)
    float command;
    command = _p * _error[1] + 
              _i * (_error[1]+_error[0]) + 
              _d * (_error[1]-_error[0]);
    return command;
}

/***************************************************/
/*************** Global variables ******************/
/***************************************************/

float rad(float a) {return a/180.0*M_PI;}
float deg(float a) {return a/M_PI*180.0;}

/*********** Servos ********************************/
Servo servos[6];
float vec_l[6][3]; // vector leg
float alpha[6];


/*********** WiiChuck ***************************/
WiiChuck chuck = WiiChuck();

int valX=0,
    valY=0;

/************** Platform ******************************/
  
// Variables related to the platform
float phi_deg   = 0.0, // roll angle in degrees
      theta_deg = 0.0, // pitch angle in degrees
      psi_deg   = 0.0; // yaw angle in degrees

float phi   = 0.0, // roll angle in radians
      theta = 0.0, // pitch angle in radians
      psi   = 0.0; // yaw angle in radians

float _phi, _theta;
// Constants related to the platform
float A = 50e-3;
float C = A/2+A/4*sqrt(2)/2;
float D = A/2+3*A/4*sqrt(2)/2;
float E = A/2*(1+sqrt(2));

float T[3];     // Translation matrix

float b[6][3] = {{  23e-3, -65e-3,   ZB},
                {   45e-3, -53e-3,   ZB},
                {   45e-3,  53e-3,   ZB},
                {   23e-3,  65e-3,   ZB},
                {  -75e-3,  12e-3,   ZB},
                {  -70e-3, -12e-3,   ZB}};

/* Coordinates of the plaform attachement points
 * In the platform framework
 */

float p[6][3] = {{ -10e-3, -55e-3,   ZP},
                 {  52e-3, -20e-3,   ZP},
                 {  52e-3,  20e-3,   ZP},
                 { -10e-3,  55e-3,   ZP},
                 { -41e-3,  38e-3,   ZP},
                 { -41e-3, -38e-3,   ZP}
                 };

// Orientation of the servos
float beta[6] = {rad(-150),
                 rad(30),
                 rad(-30),
                 rad(150),
                 rad(90),
                 rad(-90)};

/*************** Timers  ****************************/

Timer t;

/*************** TouchScreen ************************/

Adafruit_STMPE610 touch = Adafruit_STMPE610();
uint16_t stmpeX, stmpeY;
uint8_t stmpeZ;
float ballX, ballY;


/*************** Controlers ************************/

OneDimController rollCtrl( CP/1e6, CI/1e6, CD/1e6),
                 pitchCtrl(CP/1e6, CI/1e6, CD/1e6);

float targetX, targetY;

                 
/***************************************************/
/*************** Main functions ********************/
/***************************************************/

void writeInfoOnSerial()
{
  //Dump platform position
  Serial.print(ballX,5);
  Serial.print(' ');
  Serial.print(ballY,5);
  Serial.print(' ');
  Serial.print(phi,5);
  Serial.print(' ');
  Serial.println(theta,5);
  /*Serial.println(T[0],5);
  Serial.println(T[1],5);
  Serial.println(T[2],5);*/
  
  //Dump ball position
}

void setup()
{
  pinMode(modePin, INPUT);
  
  T[0] = 0;
  T[1] = 0;
  T[2] = H0;
  
  // Serial connection
  Serial.begin(serialSpeed);
  Serial.println("platjoy02");
  Serial.flush();

  // Nunchuck
  chuck.begin();
  chuck.update();

  // Touchscreen
  if (! touch.begin())
  {
    Serial.println("STMPE not found!");
    while(1);
  }

  //Serial and Timer
  t.every(serialPeriod, writeInfoOnSerial);

  // Configuring ports 2 to 8 as servo outputs
  // It is important to keep 0 and 1 unused because they are used for
  // serial communication
  for(int i=0; i<6; i++)
  {
    servos[i].attach(i+2);
  }
  
  // Definition of the target point
  targetX = stmpeX0;
  targetY = stmpeY0;
}

void loop()
{
  
  if(digitalRead(modePin) == HIGH)
  {
    /* ###### AUTOMATIC MODE ###### */
 
    /****** Reads ball position *******/
    if (touch.touched())
    {
      // read x & y & z;
      while (! touch.bufferEmpty()) {
        touch.readData(&stmpeX, &stmpeY, &stmpeZ);
      }
      touch.writeRegister8(STMPE_INT_STA, 0xFF); // reset all ints
      
      ballX = stmpeX;
      ballY = stmpeY;
      
      /*
      // Conversion to physical units
      ballX = map(stmpeX,stmpeXmin,stmpeXmax,plateXmin,plateXmax);
      ballY = map(stmpeY,stmpeYmin,stmpeYmax,plateYmin,plateYmax);
      */
      
      /****** Control *******/
  
      phi = rollCtrl.calCommand(ballX-targetX); // rad
      theta = pitchCtrl.calCommand(ballY-targetY); // rad
    }
    else
    {
      // Do not reset the position
    }
  }
  else
  {
    /* ###### Manual command with Nunchuck ###### */
    chuck.update();
    valX = chuck.readJoyX();
    valY = chuck.readJoyY();
    theta   = map(valX, joyXmin, joyXmax, -maxAnglePlate, maxAnglePlate);
    phi = map(valY, joyYmin, joyYmax, maxAnglePlate, -maxAnglePlate);
    psi = 0.0;

    psi = rad(psi);
    theta = rad(theta);
    phi = rad(phi);
  }
    



  /***** Virtual legs length ******/
  for(int i=0; i<6; i++)
  {
    vec_l[i][0] = cos(psi)*cos(theta) * p[i][0] + 
                  (-sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi)) * p[i][1] +
                  (sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi)) * p[i][2]
                  - b[i][0] + T[0];
    vec_l[i][1] = sin(psi)*cos(theta) * p[i][0] +
                  (cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi)) * p[i][1] +
                  (-cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi)) * p[i][2]
                  - b[i][1] + T[1];
    vec_l[i][2] = -sin(theta) * p[i][0] +
                  cos(theta)*sin(phi) * p[i][1] +
                  cos(theta)*cos(phi) * p[i][2]
                  -b[i][2] + T[2];
  }

  float la[] = {16.5e-3, 16.5e-3, 16.5e-3, 16.5e-3, 16.5e-3, 16.5e-3}; // length of the servo arms [mm]
  float ls[] = {93.5e-3, 93.5e-3, 93.5e-3, 93.8e-3, 93.9e-3, 93.0e-3}; // length of the push rods [mm]
  
  /***** Calculate the necessary angles of the servos ****/
  for(int i=0; i<6; i++)
  {
    /* l is the lentgh of the leg
     * It is the norm of the vector l vec_l
     */
    float l = sqrt( sq(vec_l[i][0]) +
        sq(vec_l[i][1]) +
        sq(vec_l[i][2]) );
      
    float L = sq(l)-(sq(ls[i]) - sq(la[i]));
    float M = 2*la[i]*(p[i][2]+T[2]-b[i][2]);
    float N = 2*la[i]*(cos(beta[i])*(p[i][0]+T[0]-b[i][0])+sin(beta[i])*(p[i][1]+T[1]-b[i][1]));
    alpha[i] = deg(asin(L/sqrt(sq(M)+sq(N))) - atan(N/M));

    if(isnan(alpha[i]))
    {
      /*
      Serial.println("NaN value -> BREAK");
      Serial.println("l L M N");
      Serial.println(l,5);
      Serial.println(L,5);
      Serial.println(M,5);
      Serial.println(N,5);
      Serial.print("angle_"); Serial.print(i); Serial.print(" ");
      Serial.println(alpha[i],5);
      */
      
    }
    else if (abs(alpha[i]) > maxAngleServos)
    {
      /*
      Serial.print("angle_"); Serial.print(i); Serial.print(" too large: ");
      Serial.println(alpha[i],5);
      */
    }
    else
    {
      int sign;
      // Negative angle if the servo is odd
      if(i%2>0)
        sign=1;
       else
        sign=-1;

      /*
      //Dump the servo angles
      Serial.print("angle_"); Serial.print(i); Serial.print(" ");
      Serial.println(90+sign*alpha[i]);
      */

      // Apply the servo angles
      servos[i].write(90+sign*alpha[i]);
    }
  }
}
