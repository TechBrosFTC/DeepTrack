package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "Treino Ana Laura")
public class AnaLaura extends LinearOpMode {
    DcMotorEx leftBack, rightBack, leftFront, rightFront;
    double F, D, T, E;
    void orientacaoY(double power){
        leftBack.setPower(power);
        rightBack.setPower(power);
        leftFront.setPower(power);
        rightFront.setPower(power);
    }
    void orientacaoX(double power){
        leftBack.setPower(power);
        rightBack.setPower(-power);
        leftFront.setPower(-power);
        rightFront.setPower(power);
    }
    void pare(){
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()){

            double speedForward = -gamepad1.right_stick_y;
            double speedRotation = gamepad1.right_stick_x;

            if (speedForward > 0.05){
                F = speedForward;
                T = 0;
            }else if (speedForward < -0.05){
                T = speedForward;
                F = 0;
            }else{
                pare();
            }

            if (speedRotation > 0.05){
                D = speedRotation;
                E = 0;
            }else if (speedRotation < -0.05){
                E = speedRotation;
                D=0;
            }else{
                pare();
            }

            if ((F > T) && (F > D) && (F > E)){
                orientacaoY(F);
            }else if ((T > F) && (T > D) && (T > E)){
                orientacaoY(T);
            }else if((D > F) && (D > T) && (D > E)){
                orientacaoX(D);
            }else if ((E > F) && (E > T) && (E > D)){
                orientacaoX(E);
            }else{
                pare();
            }
        }
    }
}
