
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Library Includes.                                                             *
 * Be sure to check each of these to see what variables/functions are made        *
 * global and accessible.                                                        *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "pins.h"
#include "utils.h"
#include "motors.h"
#include "pid.h"
#include "interrupts.h"
#include "kinematics.h"
#include "line_sensors.h"
#include "irproximity.h"
#include "mapping.h"
#include "RF_Interface.h"
#include <Wire.h>
#include "imu.h"
#include "magnetometer.h"
#include "Pushbutton.h"
#include "functions.h"


//test

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Definitions.  Other definitions exist in the .h files above.                  *
 * Also ensure you check pins.h for pin/device definitions.                      *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define BAUD_RATE 9600
#define DEFAULT_SPEED 20


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Class Instances.                                                              *
 * This list is complete for all devices supported in this code.                 *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
Kinematics    Pose; //Kinematics class to store position and heading

LineSensor    LineLeft(LINE_LEFT_PIN); //Left line sensor
LineSensor    LineCentre(LINE_CENTRE_PIN); //Centre line sensor
LineSensor    LineRight(LINE_RIGHT_PIN); //Right line sensor

SharpIR       DistanceSensor(SHARP_IR_PIN); //Distance sensor
SharpIR       DistanceSensor2(SHARP_IR_PIN_2); //Distance sensor

Imu           Imu;

Magnetometer  Mag; // Class for the magnetometer

Motor         LeftMotor(MOTOR_PWM_L, MOTOR_DIR_L);
Motor         RightMotor(MOTOR_PWM_R, MOTOR_DIR_R);

//These work for our Romi - We strongly suggest you perform your own tuning
PID           LeftSpeedControl( 3.5, 20.9, 0.04 );
PID           RightSpeedControl( 3.5, 20.9, 0.04 );
PID           HeadingControl( 5, 0, 0.0 );
// PID           ObstacleFollowControl( 0.08, 0.2, 0 );
PID           ObstacleFollowControl( 0.05, 0, 0 );
Mapper        Map; //Class for representing the map

Pushbutton    ButtonB( BUTTON_B, DEFAULT_STATE_HIGH);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Global variables.                                                             *
 * These global variables are not mandatory, but are used for the example loop() *
 * routine below.                                                                *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//Use these variables to set the demand of the speed controller
bool use_speed_controller = true;
bool finished_mapping_flag = false;
bool obstacle_following_flag = false;
bool jumpoff_flag = false;
float left_speed_demand = 0;
float right_speed_demand = 0;

float jumpoff_x = 0;
float jumpoff_y = 0;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * This setup() routine initialises all class instances above and peripherals.   *
 * It is recommended:                                                            *
 * - You keep this sequence of setup calls if you are to use all the devices.    *
 * - Comment out those you will not use.                                         *
 * - Insert new setup code after the below sequence.                             *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void setup()
{

  // These two function set up the pin
  // change interrupts for the encoders.
  setupLeftEncoder();
  setupRightEncoder();
  startTimer();

  //Set speed control maximum outputs to match motor
  LeftSpeedControl.setMax(100);
  RightSpeedControl.setMax(100);
  ObstacleFollowControl.setMax(100);

  // For this example, we'll calibrate only the 
  // centre sensor.  You may wish to use more.
  LineCentre.calibrate();

  //Setup RFID card
  setupRFID();

  // These functions calibrate the IMU and Magnetometer
  // The magnetometer calibration routine require you to move
  // your robot around  in space.  
  // The IMU calibration requires the Romi does not move.
  // See related lab sheets for more information.
  /*
  Wire.begin();
  Mag.init();
  Mag.calibrate();
  Imu.init();
  Imu.calibrate();
  */

  // Set the random seed for the random number generator
  // from A0, which should itself be quite random.
  randomSeed(analogRead(A0));

  
  // Initialise Serial communication
  Serial.begin( BAUD_RATE );
  delay(1000);
  Serial.println("Board Reset");

  // Romi will wait for you to press a button and then print
  // the current map.
  //
  // !!! A second button press will erase the map !!!
  ButtonB.waitForButton();  

  Map.printMap();

  // Watch for second button press, then begin autonomous mode.
  ButtonB.waitForButton();  

  Serial.println("Map Erased - Mapping Started");
  Map.resetMap();

  // Your extra setup code is best placed here:
  // ...
  // ...
  // but not after the following:

  // Because code flow has been blocked, we need to reset the
  // last_time variable of the PIDs, otherwise we update the
  // PID with a large time elapsed since the class was 
  // initialised, which will cause a big intergral term.
  // If you don't do this, you'll see the Romi accelerate away
  // very fast!
  LeftSpeedControl.reset();
  RightSpeedControl.reset();
  left_speed_demand = 5;
  right_speed_demand = 5;

  
  
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * This loop() demonstrates all devices being used in a basic sequence.          
 * The Romi should:                                                                              
 * - move forwards with random turns 
 * - log lines, RFID and obstacles to the map.
 * 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void loop() {

  // Remember to always update kinematics!!
  Pose.update();
  float distance_side = DistanceSensor.getDistanceInMM(); //* sqrtf(3)/2;
  float distance_head = DistanceSensor2.getDistanceInMM();

  //     if (distance_side < 15)
  // {
  //   obstacle_following_flag = true;
  // }
  //     if (obstacle_following_flag == true)
  //   {
  //     // left_speed_demand = 0;
  //     // right_speed_demand = 0;
  //     // delay(2000);
  //     obstacleFollow();

  //   }

    if (distance_side < 7)
  {
    use_speed_controller = true;
    obstacle_following_flag = true;
    // left_speed_demand = 0;
    // right_speed_demand = 0;
    // delay(1000);
    // rotationByDegree(-85,5);

  }
    if (obstacle_following_flag == true)
    {
          //need to avoid speed becomes 0.

      obstacleFollow();

    }
    else
    {
      doMovement();

    }






  if (finished_mapping_flag == false)
  {
    doMapping();

  }
  // else
  // {
  //   /* reserved for path planning, should be achieved by Yu Chen and Lucas */
  // }


  
  //   // 计算当前位置与起点距离
  //     float error_obstacleFollow = sqrt(pow((Pose.getX()-jumpoff_x),2) + pow((Pose.getY()-jumpoff_y),2));
  //   // 当回到起点时，退出obstacleFollow,回到螺旋线
  delay(2);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * We have implemented a random walk behaviour for you
 * with a *very* basic obstacle avoidance behaviour.  
 * It is enough to get the Romi to drive around.  We 
 * expect that in your first week, should should get a
 * better obstacle avoidance behaviour implemented for
 * your Experiment Day 1 baseline test.  
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void doMovement() {

    float distance_side = DistanceSensor.getDistanceInMM(); //* sqrtf(3)/2;
  float distance_head = DistanceSensor2.getDistanceInMM();



  // Static means this variable will keep
  // its value on each call from loop()
  use_speed_controller = false;
  static bool start_flag = true;
  static unsigned long walk_update = millis();
  static double last_degree = 0;
  static float y_offset = 3;
  static float evolve_factor = 1.0;
  static float init_x = 900;
  static float init_y = 900 + y_offset;
  double target_degree = formatDegreesRange(Pose.getGlobalDegrees(init_x, init_y) - evolve_factor + 90);
  double forward_bias = 0;
  // used to control the forward and turn
  // speeds of the robot.
  // float forward_bias;
  // float turn_bias;
  // if (start_flag = true && target_degree < 0)
  //   target_degree = 0;
  // else
  //   start_flag = false;
  

  if (abs(Pose.getThetaDegrees()) > 155 || abs(Pose.getThetaDegrees()) < 25)
    forward_bias = HeadingControl.update(target_degree - last_degree, abs(Pose.getThetaDegrees())*sign(target_degree) - last_degree);
  else
    forward_bias = HeadingControl.update(target_degree - last_degree, Pose.getThetaDegrees() - last_degree);
  left_speed_demand = constrain(DEFAULT_SPEED - forward_bias, -100, 100);
  right_speed_demand = constrain(DEFAULT_SPEED + forward_bias, -100, 100);
  last_degree = target_degree;
  if (Map.readMapFeature(Pose.getY(), Pose.getX()) != 'L'
      && Map.readMapFeature(Pose.getY(), Pose.getX()) != 'O'
      && Map.readMapFeature(Pose.getY(), Pose.getX()) != 'R') {
    Map.updateMapFeature( (byte)' ', Pose.getY(), Pose.getX());
  }
  
  // left_speed_demand = 10;
  // right_speed_demand = 10;
  // delay(20);
  // Serial.print(target_degree);
  // Serial.print(',');
  // Serial.println(Pose.getThetaDegrees());
//  Serial.println(Pose.getX()-init_x);
//  Serial.println(Pose.getY()-init_y);
//  Serial.println(Pose.getGlobalDegrees(init_x, init_y));
//  Serial.println(forward_bias);
//  Serial.println("---------");
//   rotation(-180, 10);
//   rotation(-180, 20);
//   rotation(-30, 10);
// //  Pose.printPose();
//   while(true){
//         Pose.printPose();
//         delay(100);
//   }

  // Check if we are about to collide.  If so,
  // zero forward speed
//  if( DistanceSensor.getDistanceRaw() > 450 ) {
//    forward_bias = 0;
//  } else {
//    forward_bias = 5;
//  }

  // Periodically set a random turn.
  // Here, gaussian means we most often drive
  // forwards, and occasionally make a big turn.
//  if( millis() - walk_update > 500 ) {
//    walk_update = millis();
//
//    // randGaussian(mean, sd).  utils.h
//    turn_bias = randGaussian(0, 6.5 );

    // Setting a speed demand with these variables
    // is automatically captured by a speed PID 
    // controller in timer3 ISR. Check interrupts.h
    // for more information.
//    left_speed_demand = forward_bias + turn_bias;
//    right_speed_demand = forward_bias - turn_bias;
//  } 



}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * This function groups up our sensor checks, and then
 * encodes into the map.  To get you started, we are 
 * simply placing a character into the map.  However,
 * you might want to look using a bitwise scheme to 
 * encode more information.  Take a look at mapping.h
 * for more information on how we are reading and
 * writing to eeprom memory.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void doMapping() {
  
  // Read the IR Sensor and determine distance in
  // mm.  Make sure you calibrate your own code!
  // We threshold a reading between 40mm and 12mm.
  // The rationale being:
  // We can't trust very close readings or very far.
  // ...but feel free to investigate this.
  float distance = DistanceSensor.getDistanceInMM();
  if( distance < 40 && distance > 12 ) {

    // We know the romi has the sensor mounted
    // to the front of the robot.  Therefore, the
    // sensor faces along Pose.Theta.
    // We also add on the distance of the 
    // sensor away from the centre of the robot.
    distance += 80;


    // Here we calculate the actual position of the obstacle we have detected
    float projected_x = Pose.getX() + ( distance * cos( Pose.getThetaRadians() ) );
    float projected_y = Pose.getY() + ( distance * sin( Pose.getThetaRadians() ) );
    Map.updateMapFeature( (byte)'O', projected_x, projected_y );
    
    
  } 

  // Check RFID scanner.
  // Look inside RF_interface.h for more info.
  if( checkForRFID() ) {

    // Add card to map encoding.  
    Map.updateMapFeature( (byte)'R', Pose.getY(), Pose.getX() );
    playTone(0);
    // you can check the position reference and
    // bearing information of the RFID Card in 
    // the following way:
    // serialToBearing( rfid.serNum[0] );
    // serialToXPos( rfid.serNum[0] );
    // serialToYPos( rfid.serNum[0] );
    //
    // Note, that, you will need to set the x,y 
    // and bearing information in rfid.h for your
    // experiment setup.  For the experiment days,
    // we will tell you the serial number and x y 
    // bearing information for the cards in use.  
    
  } 

  // Basic uncalibrated check for a line.
  // Students can do better than this after CW1 ;)
  if( LineCentre.readRaw() > 580 ) {
      Map.updateMapFeature( (byte)'L', Pose.getY(), Pose.getX() );
      playTone(0);
  } 
}

void obstacleFollow()
{
  
  float distance_side = DistanceSensor.getDistanceInMM(); //* sqrtf(3)/2;
  float distance_head = DistanceSensor2.getDistanceInMM();
  float bias = ObstacleFollowControl.update(7, distance_side);
  // if (jumpoff_flag == false)
  // {
  //   float jumpoff_x = Pose.getX();
  //   float jumpoff_y = Pose.getY();
  //   jumpoff_flag = true;
  // }

  if(distance_head < 5)
  {
    left_speed_demand = 0;
    right_speed_demand = 0;
    delay(1000);
    rotationByDegree(85,5);

  }
  
  // if (distance_side > 50)
  // {
  //   // float last_measure = Pose.getThetaDegrees();
  //   // float heading_bias = HeadingControl.update(-40, (Pose.getThetaDegrees()));
  //   left_speed_demand = 5;
  //   right_speed_demand = 5;
  //   delay(2800);
  //   rotationByDegree(-85, 5);
  //   //       left_speed_demand = 2 - 0.05 * bias;
  //   // right_speed_demand = 2 + 0.05 * bias;
  //   // left_speed_demand = 5 - 0.05 * heading_bias;
  //   // right_speed_demand = -5 + 0.05 * heading_bias;

  // }

if(distance_side < 50){
   left_speed_demand = 5 - 1 * bias;
    right_speed_demand = 5 + 1 * bias;

}
     
    // float error_obstacleFollow = sqrt(pow((Pose.getX()-jumpoff_x),2) + pow((Pose.getY()-jumpoff_y),2));
    // if(error_obstacleFollow < 10)
    // {
      
    // }
    else{
      while(distance_side > 50){
        distance_side = DistanceSensor.getDistanceInMM();
rotationByDirection('R',1);
      }


    }
    
}