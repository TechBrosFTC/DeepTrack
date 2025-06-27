package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.deeptrack.MecanumDrive;

public class teste extends LinearOpMode{
    DcMotorSimple direitaFrente = hardwareMap.get(DcMotorSimple.class, "motor1");
    DcMotorSimple direitaTras = hardwareMap.get(DcMotorSimple.class, "motor2");
    DcMotorSimple esquerdaFrente = hardwareMap.get(DcMotorSimple.class, "motor3");
    DcMotorSimple esquerdaTras = hardwareMap.get(DcMotorSimple.class, "motor4");
   // MecanumDrive mecanumDrive = new MecanumDrive(0, 0, 0, direitaFrente, direitaTras, esquerdaFrente, esquerdaTras, 0, 0, 0);
    public RobotSetup peach = new RobotSetup(hardwareMap);

    public void runOpMode(){
        waitForStart();
        while(opModeIsActive()){

        }
    }
}