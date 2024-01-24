#pragma once


struct pid_controller {
    float kP = 0.0;
    float kI = 0.0;
    float kD = 0.0;
    float Imax = 0.1;
    float setpoint = 0.0;

    

    float error = 0.0;
    float accumulator = 0.0;
};