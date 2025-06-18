package org.firstinspires.ftc.teamcode.deeptrack;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {
    private double x;
    private double y;
    private double theta;
    private DcMotorSimple leftFront;
    private DcMotorSimple leftBack;
    private DcMotorSimple rightFront;
    private DcMotorSimple rightBack;
    private double kp;
    private double kd;
    private double ki;
    private PIDController pid;
    private IMU imu;
    public MecanumDrive(double initial_x, double initial_y, double initial_theta, DcMotorSimple leftFront, DcMotorSimple leftBack, DcMotorSimple rightFront, DcMotorSimple rightBack, double kp, double kd, double ki){
        x = initial_x;
        y = initial_y;
        theta = initial_theta;
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;

    }
    public void update(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

}
