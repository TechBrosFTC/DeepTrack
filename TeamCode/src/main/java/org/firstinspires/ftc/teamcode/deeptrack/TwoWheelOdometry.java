package org.firstinspires.ftc.teamcode.deeptrack;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
public class TwoWheelOdometry {
    private String instance_name_x;
    private String instance_name_y;
    private boolean revt_x;
    private boolean revt_y;
    public DcMotor x;
    public DcMotor y;
    public TwoWheelOdometry(String inst_name_x, String inst_name_y, boolean revertx, boolean reverty) {
        this.instance_name_x = inst_name_x;
        this.instance_name_y = inst_name_y;
        this.revt_x = revertx;
        this.revt_y = reverty;
        x = hardwareMap.get(DcMotor.class, inst_name_x);
        y = hardwareMap.get(DcMotor.class, inst_name_y);
        if(revt_x == true){
            x.setDirection(DcMotor.Direction.REVERSE);
        }

        if(revt_y == true){
            y.setDirection(DcMotor.Direction.REVERSE);
        }
    }
    public int getX(){
        return x.getCurrentPosition();
    }
    public int getY(){
        return y.getCurrentPosition();
    }
    public void resetX(){
        x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        x.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void resetY(){
        y.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        y.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void resetOdometryEncoder(){
        x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        x.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        y.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        y.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
