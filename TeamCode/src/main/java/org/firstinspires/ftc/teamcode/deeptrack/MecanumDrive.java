package org.firstinspires.ftc.teamcode.deeptrack;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.DriveConstants;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**This is the main class for movimentation. There are methods for movimentation on xy axis and turns,
localization of the robot and the route calculation built-in in the functions*/
public class MecanumDrive {//Class mecanum drive
    public double x;//Set of variables for localization
    public double y;
    public double theta;
    public DriveTrain driveTrain;//Drive train
    private double kp;//pid constants
    private double kd;
    private double ki;
    private double direction;
    private double speed;
    private double target;
    private double proportional;
    private double derivative;
    private double integral;
    private double previousError;
    private double error;
    private RevHubOrientationOnRobot orientation;//orientation of the robot
    private double divisor = 100;
    public IMU imu;//Inertial measuremennt unit
    public Odometry odometry;//Odometry
    public MecanumDrive(double initial_x, double initial_y, double initial_theta, DriveTrain driveTrain, double kp, double kd, double ki, IMU imu, Odometry odometry){
        x = initial_x;
        y = initial_y;
        theta = initial_theta;
        this.driveTrain = driveTrain;
        this.kp = kp;
        this.kd = kd;
        this.ki = ki;
        this.odometry = odometry;
        this.imu = imu;
    }

    public void turn(double target, double max_speed, double min_speed){
        
    }
    public void MRUVY(double min_speed, double max_speed, double distance, double theta_target){
        int ticks = (int) (distance / DriveConstants.CM_PER_TICK);
        double a = Math.abs(max_speed - min_speed)/(ticks*0.3);
        double b = Math.abs(min_speed);
        odometry.resetY();
        if (min_speed > 0) {
            while (odometry.getY() < ticks*0.7){
                error = (target - imu.getRobotYawPitchRollAngles().getYaw());
                proportional = error*kp;
                derivative = (error - previousError)*kd;
                integral = error+ki;
                previousError = error;
                direction = (proportional + derivative + integral)/divisor;
                driveTrain.drive(max_speed, direction);
            }
            while (odometry.getY() < ticks){
                error = (target - imu.getRobotYawPitchRollAngles().getYaw());
                proportional = error*kp;
                derivative = (error - previousError)*kd;
                integral = error+ki;
                previousError = error;
                direction = (proportional + derivative + integral)/divisor;
                speed = a*(this.odometry.getY()-ticks*0.7) + b;
                driveTrain.drive(speed, direction);
            }
            driveTrain.stop();
        }else{
            while (odometry.getY() > -ticks*0.7) {
                error = (imu.getRobotYawPitchRollAngles().getYaw() - target);
                proportional = error * kp;
                derivative = (error - previousError) * kd;
                integral = error + ki;
                previousError = error;
                direction = (proportional + derivative + integral)/divisor;
                driveTrain.drive(max_speed, direction);
            }
            while (odometry.getY() > -ticks) {
                error = (imu.getRobotYawPitchRollAngles().getYaw()- target);
                proportional = error * kp;
                derivative = (error - previousError) * kd;
                integral = error + ki;
                previousError = error;
                direction = (proportional + derivative + integral)/divisor;
                speed = a * (this.odometry.getY() - (ticks*0.7)) + b;
                driveTrain.drive(speed, direction);
            }
            driveTrain.stop();
        }
    }
    public void parar(){
        driveTrain.stop();
    }



}
