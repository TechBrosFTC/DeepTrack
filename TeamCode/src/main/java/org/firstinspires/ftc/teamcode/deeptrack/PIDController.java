package org.firstinspires.ftc.teamcode.deeptrack;

public  class PIDController {
    double kp;
    double kd;
    double ki;
    double  prevError;
    double error;
    double integral;
    double proportional;
    double derivative;
    double output;

    public PIDController(double kp, double kd, double ki){
        this.kp = kp;
        this.kd = kd;
        this.ki = ki;
    }
    public double calculate(double error, double target){
        this.error = target - error;
        proportional = this.kp * error;
        derivative = this.kd * (error - prevError);
        integral += this.ki * error;
        output = proportional + derivative + integral;
        prevError = error;
        return output;
    }

}
