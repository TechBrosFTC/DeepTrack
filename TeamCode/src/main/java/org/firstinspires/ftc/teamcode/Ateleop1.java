package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name = "Ateleop1")
public class Ateleop1 extends LinearOpMode {
    private IMU imu;
    private DcMotorEx direitaFrente, direitaTras, esquerdaFrente, esquerdaTras;//instancia dos motores
    double power = 1, curvapower = 0.4, multiplicadorx = 1, multiplicador = 0.6, multiplicadorcurva = 1;//variáveis de movimentação
    double proporcional, derivativa, integral, erro, direcao = 0, ultimoerro = 0, alvo = 0; //Variáveis de cáluclo do PID
    boolean curva = false, andando = false;//Controle de movimento
    double kp = 2.2, kd = 2, ki = 0;//Constantes PID movimentação
    double lastpower = 0; //controle de movimento
    float f,t,d,e;//Direções do joystick
    boolean modolento = false;//controle de movimento
    ///////////////////////////////////////////////////////////////////////

    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        direitaFrente = hardwareMap.get(DcMotorEx.class, "rightFront");
        direitaTras = hardwareMap.get(DcMotorEx.class, "rightBack");
        esquerdaFrente = hardwareMap.get(DcMotorEx.class, "leftFront");
        esquerdaTras = hardwareMap.get(DcMotorEx.class, "leftBack");
        RevHubOrientationOnRobot revOrientaion = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revOrientaion));
        direitaFrente.setDirection(DcMotorSimple.Direction.REVERSE);
        direitaTras.setDirection(DcMotorSimple.Direction.FORWARD);
        esquerdaTras.setDirection(DcMotorSimple.Direction.FORWARD);
        esquerdaFrente.setDirection(DcMotorSimple.Direction.FORWARD);
        imu.resetYaw();
        Thread telemetria = new Thread(() -> {
            while (opModeIsActive()) {
///////////////////////////////////////////////////Telemetria/////////////////////////////////////////////////////////////////////
                telemetry.addData("giro", gyro());
                telemetry.addData("direcao:", direcao);
                telemetry.addData("controle y:", gamepad1.left_stick_y);
                telemetry.addData("controle x:", gamepad1.left_stick_x);
                telemetry.addData("power:", power);
                telemetry.addData("modolento", modolento);
                telemetry.update();
            }
        });//Thread telemetria

        waitForStart();

        imu.resetYaw();
        telemetria.start();
        while (opModeIsActive()) {
            if (gamepad1.left_stick_y != 0) {
                if (gamepad1.left_stick_y > 0) {
                    f = 0;
                    t = Math.abs(gamepad1.left_stick_y);
                } else {
                    f = Math.abs(gamepad1.left_stick_y);
                    t = 0;
                }
            } else {
                f = 0;
                t = 0;
            }
            if (gamepad1.left_stick_x != 0) {
                if (gamepad1.left_stick_x > 0) {
                    e = 0;
                    d = Math.abs(gamepad1.left_stick_x);
                } else {
                    e = Math.abs(gamepad1.left_stick_x);
                    d = 0;
                }
            } else {
                d = 0;
                e = 0;
            }

            if (f > t && f > d && f > e) { //andar para frente
                if (!andando) {
                    imu.resetYaw();
                }
                erro = alvo - gyro();
                proporcional = erro * kp;
                derivativa = (erro - ultimoerro) * kd;
                integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                movDirecionado(Math.abs(gamepad1.left_stick_y) * multiplicador, direcao);
                ultimoerro = erro;
                andando = true;
            }
            if (t > f && t > d && t > e) { //andar para trás
                if (!andando) {
                    imu.resetYaw();
                }
                erro = alvo - gyro();
                proporcional = erro * kp;
                derivativa = (erro - ultimoerro) * kd;
                integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                lastpower = gamepad1.left_stick_y * multiplicador;
                movDirecionado(Math.abs(gamepad1.left_stick_y) * multiplicador * -1, direcao);
                ultimoerro = erro;
                andando = false;
            }
            if (d > f && d > t && d > e) { //andar para Direita
                if (!andando) {
                    imu.resetYaw();
                }
                erro = alvo - gyro() ;
                proporcional = erro * 1;
                derivativa = (erro - ultimoerro) * 1;
                integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                mecanumdireitabase(Math.abs(gamepad1.left_stick_x), -direcao);
                andando = true;
            }
            if (e > f && e > t && e > d) { //andar para Esquerda
                if (!andando) {
                    imu.resetYaw();
                }
                erro = alvo - gyro() ;
                proporcional = erro * 1;
                derivativa = (erro - ultimoerro) * 1;
                integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                mecanumesquerdabase(Math.abs(gamepad1.left_stick_x), direcao);
                andando = true;
            }
            if (f == 0 && t == 0 && d == 0 && e == 0) {
                parar();
                andando = false;
            }
            if (gamepad1.right_stick_x > 0) {
                direitaFrente.setPower(curvapower * multiplicadorcurva * -1);
                direitaTras.setPower(curvapower * multiplicadorcurva * -1);
                esquerdaFrente.setPower(curvapower * multiplicadorcurva);
                esquerdaTras.setPower(curvapower * multiplicadorcurva);
                curva = true;
            } else if (gamepad1.right_stick_x < 0) {
                direitaFrente.setPower(curvapower * multiplicadorcurva);
                direitaTras.setPower(curvapower * multiplicadorcurva);
                esquerdaFrente.setPower(curvapower * -1 * multiplicadorcurva);
                esquerdaTras.setPower(curvapower * -1 * multiplicadorcurva);
                curva = true;
            } else {
                if (curva) {
                    parar();
                    sleep(50);
                    imu.resetYaw();
                    curva = false;
                    andando = false;
                }
            }
            if(gamepad1.cross){
                multiplicadorx = 0.5;
                multiplicador = 0.5;
                multiplicadorcurva = 0.5;
                modolento = true;
                sleep(100);
            }else{
                multiplicadorx = 1;
                multiplicador = 0.8;
                multiplicadorcurva = 1;
                modolento = false;
                sleep(100);
            }
        }
    }
    /////////////////////////////////////////////////////funções///////////////////////////////////////////////////////////////////////
    public void mecanumesquerdabase(double power, double direcao){
        if (direcao < 0){
            direitaFrente.setPower(power);
            direitaTras.setPower((power-direcao)*-1);
            esquerdaFrente.setPower(power*-1);
            esquerdaTras.setPower((power-direcao));
        }else{
            direitaFrente.setPower(power);
            direitaTras.setPower((power-direcao)*-1);
            esquerdaFrente.setPower(power*-1);
            esquerdaTras.setPower((power-direcao));
        }
    }
    public void mecanumdireitabase(double power, double direcao){
        if (direcao < 0){
            direitaFrente.setPower((power-direcao)*-1);
            direitaTras.setPower(power);
            esquerdaFrente.setPower((power-direcao));
            esquerdaTras.setPower(power*-1);
        }else{
            direitaFrente.setPower(power*-1);
            direitaTras.setPower((power-direcao));
            esquerdaFrente.setPower(power);
            esquerdaTras.setPower((power-direcao)*-1);
        }
    }
    public void movDirecionado(double power, double direcao){
        if (power < 0) {
            if (direcao >= 0){
                direitaFrente.setPower(power+direcao);
                direitaTras.setPower(power+direcao);
                esquerdaFrente.setPower(power);
                esquerdaTras.setPower(power);
            }else{
                direitaFrente.setPower(power);
                direitaTras.setPower(power);
                esquerdaFrente.setPower(power-direcao);
                esquerdaTras.setPower(power-direcao);
            }
        }else{
            if (direcao >= 0) {
                direitaFrente.setPower(power);
                direitaTras.setPower(power);
                esquerdaFrente.setPower(power-direcao);
                esquerdaTras.setPower(power-direcao);

            } else {
                direitaFrente.setPower(power+direcao);
                direitaTras.setPower(power+direcao);
                esquerdaFrente.setPower(power);
                esquerdaTras.setPower(power);
            }
        }
    }
    /*public void resetarEncoderOdometria() {
        odometriaX.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odometriaX.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        odometriaY.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odometriaY.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }*/
    public void parar(){
        direitaFrente.setPower(0);
        direitaTras.setPower(0);
        esquerdaFrente.setPower(0);
        esquerdaTras.setPower(0);
    }
    public double gyro(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}