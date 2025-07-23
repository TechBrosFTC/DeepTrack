package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.deeptrack.MecanumDrive;
import org.firstinspires.ftc.teamcode.deeptrack.PIDController;
@Autonomous (name="Auto")
public class Auto extends LinearOpMode{

    DcMotorEx rightFront;
    DcMotorEx rightBack;
    DcMotorEx leftBack;
    DcMotorEx leftFront;
    DcMotorEx par1;
    DcMotorEx perp;
   // MecanumDrive mecanumDrive = new MecanumDrive(0, 0, 0, direitaFrente, direitaTras, esquerdaFrente, esquerdaTras, 0, 0, 0);
    public void runOpMode(){
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        par1 = hardwareMap.get(DcMotorEx.class, "par");
        perp = hardwareMap.get(DcMotorEx.class, "perp");
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        RobotSetup peach = new RobotSetup(hardwareMap, rightFront, rightBack, leftFront, leftBack, par1, perp);

        waitForStart();

        while(opModeIsActive()){
            peach.imu.resetYaw();
            peach.mecanumDrive.MRUVY(0.2, 0.8, 50, 0);
        }
    }
}