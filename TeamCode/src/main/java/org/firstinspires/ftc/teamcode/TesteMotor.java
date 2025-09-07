package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name="TesteMotor")

public class TesteMotor extends LinearOpMode{
    private DcMotorEx motor;
    @Override
    public void runOpMode(){
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a){
                motor.setPower(1);
            }else if(gamepad1.b){
                motor.setPower(-1);
            }else{
                motor.setPower(0);
            }
        }
    }

}
