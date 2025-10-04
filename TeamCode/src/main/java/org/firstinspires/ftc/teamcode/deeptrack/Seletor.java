package org.firstinspires.ftc.teamcode.deeptrack;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Seletor")
public class Seletor extends LinearOpMode{
    Servo seletor;
    @Override
    public void runOpMode(){
        seletor = hardwareMap.get(Servo.class, "seletor");
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.a){
                seletor.setPosition(1.0);
            }else if(gamepad1.b) {
                seletor.setPosition(0.0);
            }else{
                seletor.setPosition(0.5);
            }
        }
    }
}