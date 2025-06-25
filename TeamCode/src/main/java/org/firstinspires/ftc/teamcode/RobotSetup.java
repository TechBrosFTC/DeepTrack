package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.teamcode.deeptrack.DriveTrain;
import org.firstinspires.ftc.teamcode.deeptrack.MecanumDrive;
import org.firstinspires.ftc.teamcode.deeptrack.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.deeptrack.InertialMeasurementUnit;
/**
 *This is the class where the robot setup is initialized. You need to make sure you assigned the right names
 * to the motors and sensors. Select the right kind of odometry you have.
 * @author Gabriel Borges
 */
public class RobotSetup {
    /*
    public TwoWheelOdometry odometry;  //You need to delete this line if you're using a three wheel odometry
    public ThreeWheelOdometry odometry;//Same thing here
     */
    public RobotSetup(HardwareMap hardwareMap) {
        /*
        odometry = new TwoWheelOdometry(hardwareMap,false, false);
        odometry = new ThreeWheelOdometry(hardwareMap,true, false);
        odometry = false;
        */
    }
    public DriveTrain driveTrain = new DriveTrain(hardwareMap.get(DcMotorSimple.class, "leftFront"), hardwareMap.get(DcMotorSimple.class, "leftBack"), hardwareMap.get(DcMotorSimple.class, "rightFront"), hardwareMap.get(DcMotorSimple.class, "rightBack"));
    RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
    public InertialMeasurementUnit imu = new InertialMeasurementUnit(orientation, hardwareMap, "imu");
    public MecanumDrive mecanumDrive = new MecanumDrive(0, 0, 0, driveTrain, 0, 0, 0, imu);

}
