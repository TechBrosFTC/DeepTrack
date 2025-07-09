package org.firstinspires.ftc.teamcode.deeptrack;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Odometry {
    public double x;
    public double y;
    public double theta;
    private int odometryType = 0;
    private Encoder par1;
    private Encoder par2;
    private Encoder perp;
    public Odometry(double initial_x, double initial_y, double initial_theta, Encoder par1, Encoder par2, Encoder perp){
        x = initial_x;
        y = initial_y;
        theta = initial_theta;
        this.par1 = par1;
        this.par2 = par2;
        this.perp = perp;
        odometryType = 0;
    }
    public Odometry(double initial_x, double initial_y, double initial_theta, Encoder par1, Encoder perp){
        x = initial_x;
        y = initial_y;
        theta = initial_theta;
        this.par1 = par1;
        this.par2 = null;
        this.perp = perp;
        odometryType = 1;
    }
    public void resetX(){
        perp.resetEncoder();
        x = 0;
    }
    public void resetY(){
        switch (odometryType) {
            case 0:
                par1.resetEncoder();
                par2.resetEncoder();
                break;
            case 1:
                par1.resetEncoder();
                break;
        }
    }
    public double getY(){
        double value;
        switch (odometryType) {
            case 0:
                value = Math.round((perp.getEncoder() + par1.getEncoder()) / 2);
                break;
            case 1:
                value = par1.getEncoder();
                break;
            default:
                value = 0;
                break;
        }
        return value;
    }
    public double getX() {
        return perp.getEncoder();
    }
}
