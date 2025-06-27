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
}
