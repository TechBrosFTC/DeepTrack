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
    private PIDController pid;
    private RevHubOrientationOnRobot orientation;//orientation of the robot
    public IMU imu;//Inertial measuremennt unit
    public MecanumDrive(double initial_x, double initial_y, double initial_theta, DriveTrain driveTrain, double kp, double kd, double ki, InertialMeasurementUnit imu){
        x = initial_x;
        y = initial_y;
        theta = initial_theta;
        this.driveTrain = driveTrain;
        this.kp = kp;
        this.kd = kd;
        this.ki = ki;
        pid = new PIDController(kp, kd, ki);
        this.imu = imu.imu;
    }

    public void turn(double target, double max_speed, double min_speed){
        
    }
    private double getIMU(){
        return imu.getRobotYawPitchRollAngles().getYaw();
    }
    public void update(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

}
