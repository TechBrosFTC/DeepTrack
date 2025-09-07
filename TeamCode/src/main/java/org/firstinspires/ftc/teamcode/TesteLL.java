package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "testeLL")

public class TesteLL extends OpMode {
    private Limelight3A limelight;
    private Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void start(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start(); // This tells Limelight to start looking!
    }
    //-23 = 0, 23 = 0.65

    @Override
    public void loop(){
        //range do servo vai de 0 a 0.65, 1 posição recolhida
        LLResult result = limelight.getLatestResult();

        double tx = result.getTx(); // How far left or right the target is (degrees)
        double ty = result.getTy(); // How far up or down the target is (degrees)
        double ta = result.getTa(); // How big the target looks (0%-100% of the image)

        telemetry.addData("Target X", tx);
        telemetry.addData("Target Y", ty);
        telemetry.addData("Target Area", ta);
        telemetry.update();
    }

}
