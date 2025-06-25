package org.firstinspires.ftc.teamcode.deeptrack;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 *This is the class for robots with only two odometry wheels. This kind of track also needs
 * a IMU sensor, built in or external. There are methods for localization of the robot.
 *@author Gabriel Borges
 */
public class TwoWheelOdometry {
    private String instance_name_x;
    private String instance_name_y;
    private boolean revt_x;
    private boolean revt_y;
    private DcMotor x;
    private DcMotor y;
    public TwoWheelOdometry(HardwareMap hardwareMap, boolean revertx, boolean reverty) {
        this.revt_x = revertx;
        this.revt_y = reverty;
        this.x = hardwareMap.get(DcMotor.class, "perp");
        this.y = hardwareMap.get(DcMotor.class, "par");
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
