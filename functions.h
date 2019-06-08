#define sign(x) ((x> 0)? 1 : ((x <0)? -1 : 0))
extern Kinematics   Pose;
// extern float left_speed_demand = 0;
// extern float right_speed_demand = 0;

bool isAchieveGoalAngle(long last_encoder_left, long last_encoder_right, float delta_angle) {

    if (abs((float)((right_encoder_count - last_encoder_right)-(left_encoder_count -last_encoder_left))/16.0 
                * Pose.angle_calibration_factor - delta_angle) <= 0.1) {
        return true;
    }
    return false;

}

void rotationByDegree(float degree, int speed) {
    // Pose.update();
    // float start_degree = (float)Pose.getThetaDegrees();
    bool control_mode = use_speed_controller;
    use_speed_controller = true;
    volatile long last_encoder_left = left_encoder_count;
    volatile long last_encoder_right = right_encoder_count;
    while (!isAchieveGoalAngle(last_encoder_left, last_encoder_right, degree)) {
        if (degree >= 0) {
                left_speed_demand = -speed;
                right_speed_demand = speed;
        }
        else {
                left_speed_demand = speed;
                right_speed_demand = -speed;
        }
        Pose.update();
        // Serial.println(Pose.getThetaDegrees());
        // Serial.println(degree);
        // Serial.println(start_degree);
        // Serial.println(fabs(Pose.getThetaDegrees() - start_degree - fmod(degree, 360) - 360));
        // Serial.println("---------------");
    }
    left_speed_demand = 0;
    right_speed_demand = 0;
    use_speed_controller = control_mode;
}

void rotationByDirection(byte direction, int speed) {
    if (direction == 'L') {
        left_speed_demand = -speed;
        right_speed_demand = speed;
    }
    else if (direction == 'R') {
        left_speed_demand = speed;
        right_speed_demand = -speed;
    }
    else
    {
        left_speed_demand = 0;
        right_speed_demand = 0;
    }
}

double formatDegreesRange (double degree)
{
    if (degree < -180) {
        degree += 360;
    }
    else if (degree > 180) {
        degree -= 360;
    }
    
    return degree;
}

void playTone(int time) {
    analogWrite(BUZZER_PIN, 127);
    delay(time);
    analogWrite(BUZZER_PIN, 0);
}