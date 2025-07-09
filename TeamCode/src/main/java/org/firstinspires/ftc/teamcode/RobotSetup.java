package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.deeptrack.DriveTrain;
import org.firstinspires.ftc.teamcode.deeptrack.Encoder;
import org.firstinspires.ftc.teamcode.deeptrack.MecanumDrive;
import org.firstinspires.ftc.teamcode.deeptrack.Odometry;
import org.firstinspires.ftc.teamcode.deeptrack.InertialMeasurementUnit;
/**
 *This is the class where the robot setup is initialized. You need to make sure you assigned the right names
 * to the motors and sensors. Select the right kind of odometry you have.
 * @author Gabriel Borges
 */
public class RobotSetup {

    public RobotSetup(HardwareMap hardwareMap) {

    }
    /** Set the encoders and the odometry. If your robot does not fit 3 odometrys, delete the par 2 line and the par 2 argument on the odometry instance.*/
    public Encoder par1 = new Encoder("par1", false, hardwareMap);
    public Encoder par2 = new Encoder("par2", false, hardwareMap);
    public Encoder perp = new Encoder("perp", false, hardwareMap);
    public Odometry odometry = new Odometry(0, 0, 0, par1, par2, perp);

    public DriveTrain driveTrain = new DriveTrain(hardwareMap.get(DcMotorSimple.class, "leftFront"), hardwareMap.get(DcMotorSimple.class, "leftBack"), hardwareMap.get(DcMotorSimple.class, "rightFront"), hardwareMap.get(DcMotorSimple.class, "rightBack"));
    RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
    public InertialMeasurementUnit imu = new InertialMeasurementUnit(orientation, hardwareMap, "imu");
    public MecanumDrive mecanumDrive = new MecanumDrive(0, 0, 0, driveTrain, 0, 0, 0, imu, odometry );
}
