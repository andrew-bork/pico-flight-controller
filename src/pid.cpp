#include <pid.hpp>

float pid_controller::update(float current, float dt) {
    float error = setpoint - current;
    float d_error = (error - previous_error) / dt;
    previous_error = error;
    accumulator += error * dt;
    
    if(accumulator > accumulator_max) {
        accumulator = accumulator_max;
    }else if(accumulator < -accumulator_max) {
        accumulator = -accumulator_max;
    }

    return kP * error + kI * accumulator /* + kD * d_error */;
}