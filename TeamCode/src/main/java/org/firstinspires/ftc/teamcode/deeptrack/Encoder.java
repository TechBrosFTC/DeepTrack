package org.firstinspires.ftc.teamcode.deeptrack;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Encoder {
    private DcMotorEx encoder;
    public Encoder(String encoder, boolean reverse, HardwareMap hardwareMap){
        this.encoder = hardwareMap.get(DcMotorEx.class, encoder);
        if(reverse){
            encoder.setDirection(DcMotorEx.Direction.REVERSE);
        }
    }
    public double getEncoder(){
        return encoder.getCurrentPosition();
    }
    public void resetEncoder(){
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}
