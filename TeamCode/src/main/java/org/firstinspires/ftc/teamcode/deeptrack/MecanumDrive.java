package org.firstinspires.ftc.teamcode.deeptrack;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    private PIDController drivePID;
    private RevHubOrientationOnRobot orientation;//orientation of the robot
    public InertialMeasurementUnit imu;//Inertial measuremennt unit
    public Odometry odometry;//Odometry
    public MecanumDrive(double initial_x, double initial_y, double initial_theta, DriveTrain driveTrain, double kp, double kd, double ki, InertialMeasurementUnit imu, Odometry odometry){
        x = initial_x;
        y = initial_y;
        theta = initial_theta;
        this.driveTrain = driveTrain;
        this.kp = kp;
        this.kd = kd;
        this.ki = ki;
        this.odometry = odometry;
        drivePID = new PIDController(kp, kd, ki);
        this.imu = imu;
    }

    public void turn(double target, double max_speed, double min_speed){
        
    }
    public void MRUVY(double min_speed, double max_speed, double distance, double theta_target){
        int ticks = (int) (distance / DriveConstants.CM_PER_TICK);
        double a = Math.abs(max_speed - min_speed)/ticks;
        double b = Math.abs(min_speed);
        this.odometry.resetY();
        if (min_speed > 0) {
            while (this.odometry.getY() < ticks*0.7){
                error = (target - this.imu.getYaw());
                proportional = error*kp;
                derivative = (error - previousError)*kd;
                integral = error+ki;
                previousError = error;
                direction = proportional + derivative + integral;
                driveTrain.drive(max_speed, derivative);
            }
            while (this.odometry.getY() < ticks){
                error = (target - this.imu.getYaw());
                proportional = error*kp;
                derivative = (error - previousError)*kd;
                integral = error+ki;
                previousError = error;
                direction = proportional + derivative + integral;
                speed = a*(Math.abs(this.odometry.getY())-ticks) + b;
                driveTrain.drive(speed, derivative);
            }
        }else{
            while (this.odometry.getY() > -ticks*0.7) {
                error = (this.imu.getYaw() - target);
                proportional = error * kp;
                derivative = (error - previousError) * kd;
                integral = error + ki;
                previousError = error;
                direction = proportional + derivative + integral;
                driveTrain.drive(max_speed, derivative);
            }
            while (this.odometry.getY() > -ticks) {
                error = (this.imu.getYaw()- target);
                proportional = error * kp;
                derivative = (error - previousError) * kd;
                integral = error + ki;
                previousError = error;
                direction = proportional + derivative + integral;
                speed = a * (Math.abs(this.odometry.getY()) - ticks) + b;
                driveTrain.drive(speed, derivative);
            }
        }
    }

    public void update(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

}
