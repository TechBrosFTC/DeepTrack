package org.firstinspires.ftc.teamcode.deeptrack;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ThreeWheelOdometry {
    private boolean revt_x;
    private boolean revt_y1;
    private boolean revt_y2;
    private DcMotor x;
    private DcMotor y1;
    private DcMotor y2;
    public ThreeWheelOdometry(HardwareMap hardwareMap, boolean revertx, boolean reverty1, boolean reverty2) {
        this.revt_x = revertx;
        this.revt_y1 = reverty1;
        this.revt_y2 = reverty2;
        this.x = hardwareMap.get(DcMotor.class, "perp");
        this.y1 = hardwareMap.get(DcMotor.class, "par1");
        this.y2 = hardwareMap.get(DcMotor.class, "par2");
        if(revt_x == true){
            x.setDirection(DcMotor.Direction.REVERSE);
        }
        if(revt_y1 == true){
            y1.setDirection(DcMotor.Direction.REVERSE);
        }
        if(revt_y2 == true){
            y2.setDirection(DcMotor.Direction.REVERSE);
        }

    }
    public int getX(){
        return x.getCurrentPosition();
    }
    public int getY(){
        return (y1.getCurrentPosition() + y2.getCurrentPosition())/2;
    }
    public double getAngle(){
        return (y1.getCurrentPosition() - y2.getCurrentPosition())/DriveConstants.TRACK_WIDTH;
    }
    public void resetX(){
        x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        x.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void resetY(){
        y1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        y1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        y2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        y2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void resetOdometryEncoder(){
        x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        x.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        y1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        y1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        y2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        y2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
