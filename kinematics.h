#ifndef _Kinematics
#define _Kinematics_h


#define GEAR_RATIO 120.0
#define COUNTS_PER_SHAFT_REVOLUTION 12.0
const float WHEEL_RADIUS = 35.0;
const float WHEEL_DISTANCE = 140;
const float COUNTS_PER_WHEEL_REVOLUTION = GEAR_RATIO * COUNTS_PER_SHAFT_REVOLUTION;
const float MM_PER_COUNT = ( 2 * WHEEL_RADIUS * PI ) / COUNTS_PER_WHEEL_REVOLUTION;

class Kinematics
{
     public:

         void  update();
         double getThetaDegrees();
         double getThetaRadians();
         double getGlobalDegrees(float init_x, float init_y);
         double getGlobalRadians(float init_x, float init_y);
         double getX();
         double getY();
         void  resetPose();
         void  setPose(double X, double Y, double theta);
         void  printPose();
         void  setDebug(bool state);
         double getDistanceFromOrigin();
         double getAngularVelocity();
         float angle_calibration_factor = 0.9923;
        
    private:

         double x=900;
         double y=900;
         double theta=0;
         double last_theta = 0;
         double angular_velocity = 0;
         long  last_left_encoder_count = 0;
         long  last_right_encoder_count = 0;
         bool  debug=false;
         unsigned long last_update = 0;

};


void Kinematics::update()
{
    //Calculate delta since last update
    double left_delta = (left_encoder_count - last_left_encoder_count)*MM_PER_COUNT;
    double right_delta = (right_encoder_count - last_right_encoder_count)*MM_PER_COUNT;
    double mean_delta = (left_delta + right_delta) / 2;  
    
    //Store counts
    last_left_encoder_count = left_encoder_count;
    last_right_encoder_count = right_encoder_count;  

    //Update position
    x+= mean_delta * cos(theta);
    y+= mean_delta * sin(theta);
    //  theta +=  (left_delta-right_delta) / (WHEEL_DISTANCE); 
    theta = deg2rad((double)(right_encoder_count - left_encoder_count ) / 16 * angle_calibration_factor); 

    double time_elapsed = millis() - last_update;
    last_update = millis();

    angular_velocity = ( (left_delta-right_delta) / WHEEL_DISTANCE );
    angular_velocity -= last_theta;
    angular_velocity /= time_elapsed;
    

     //Wrap theta between -PI and PI.
    while (theta > PI)
    {
        theta -=2*PI;
    }
    while(theta < -PI)
    {
        theta += 2*PI;
    } 

    last_theta = theta;

    if (debug)
    {
        printPose();
    }
  
}


double Kinematics::getThetaDegrees()
{
    return rad2deg(theta);
}

double Kinematics::getThetaRadians()
{
    return (theta);
}

double Kinematics::getGlobalDegrees(float init_x, float init_y)
{
    double degree = rad2deg(atan2(y - init_y, x - init_x));
    if (degree > 90)
        degree = degree - 360;
    return degree;
}

double Kinematics::getGlobalRadians(float init_x, float init_y)
{
    double degree = atan2(y - init_y, x - init_x);
    if (degree > PI/2)
        degree = degree - 2*PI;
    return degree;
}

double Kinematics::getAngularVelocity() {
    return rad2deg( angular_velocity );
}


double Kinematics::getX()
{
    return x;
}


double Kinematics::getY()
{
    return y;
}


void Kinematics::resetPose()
{

    x = 0;
    y = 0;
    theta = 0;

}


void Kinematics::setPose(double newX, double newY, double newTheta)
{

    x = newX;
    y = newY;
    theta = newTheta;

}

void Kinematics::printPose()
{

    Serial.print(F("X: "));
    Serial.print(x);
    Serial.print(F(" Y: "));
    Serial.print(y);
    Serial.print(F(" H: "));
    Serial.println(rad2deg(theta));

}


void Kinematics::setDebug(bool state)
{
    debug = state;
}


double Kinematics::getDistanceFromOrigin()
{
    return sqrt(x*x + y*y);
}

#endif
