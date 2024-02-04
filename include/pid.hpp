#pragma once


struct pid_controller {
    float kP = 0.0;
    float kI = 0.0;
    float kD = 0.0;
    float accumulator_max = 0.1;
    float setpoint = 0.0;

    

    float previous_error = 0.0;
    float accumulator = 0.0;

    float update(float current, float dt);
};