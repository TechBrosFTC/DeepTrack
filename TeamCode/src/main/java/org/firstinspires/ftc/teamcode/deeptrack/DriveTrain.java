package org.firstinspires.ftc.teamcode.deeptrack;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveTrain {
    public DcMotorSimple leftFront;
    public DcMotorSimple leftBack;
    public DcMotorSimple rightFront;
    public DcMotorSimple rightBack;
    public DriveTrain(DcMotorSimple leftFront, DcMotorSimple leftBack, DcMotorSimple rightFront, DcMotorSimple rightBack){
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
    }
    public void drive(double power, double steering){//positive power = forward, negative power = backward, positive steering = right, negative steering = left
        leftFront.setPower(power + steering);
        leftBack.setPower(power + steering);
        rightFront.setPower(power - steering);
        rightBack.setPower(power - steering);
    }
    public void turn(double power){//positive = right, negative = left
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(-power);
        rightBack.setPower(-power);
    }
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
