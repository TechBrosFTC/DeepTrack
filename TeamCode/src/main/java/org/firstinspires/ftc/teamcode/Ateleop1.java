package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

@TeleOp (name = "Ateleop1")
public class Ateleop1 extends LinearOpMode {
    private IMU imu;
    private DcMotorEx direitaFrente;
    private DcMotorEx direitaTras;
    private DcMotorEx esquerdaFrente;
    private DcMotorEx esquerdaTras;

    private DcMotorEx odometriaX;
    private DcMotorEx odometriaY;



    double power = 1;
    double powerx;
    double erro;
    double proporcional;
    double proximaposicao;
    double derivativa;
    double integral;
    double direcao = 0;
    double ultimoerro = 0;
    double curvapower = 0.4;
    double alvo = 0;
    double multiplicadorx = 1;
    boolean curva = false;
    double kp = 2.2; //5 Bom, 6 Legal-+
    double kd = 2; // 35 Fica pouco, 40 Fica Pouco, 45 Quase lá
    double ki = 0;
    String state = "";
    boolean extensor = false;
    double lastpower = 0;
    float f,t,d,e;
    double multiplicador = 0.6;
    double movetime = 0;
    double currentpower = 0;
    double fator = 0;
    //////////////////////////////////////////////////////////////////Norte Abosluto/////////////////////////////////////////////////////////
    double giroabsoluto;
    int direcaoabsoluta;
    int index;
    float maior;
    String direcaomovimento = "";
    ///////////////////////////////////////////////////////////////////////
    double joystick = 0;

    boolean modolento = false;
    int numerodeciclos = 0;
    double posicaoatualgarra = 0;
    double proximaposicaogarra = 0;
    boolean servosample = false;
    boolean servoclip = false;
    double viperPower;
    double multiplicadorcurva = 1;
    float[] stickmovimento = new float[4];
    String dir = "";
    boolean abrir = false;
    @Override

    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        direitaFrente = hardwareMap.get(DcMotorEx.class, "rightFront");
        direitaTras = hardwareMap.get(DcMotorEx.class, "rightBack");
        esquerdaFrente = hardwareMap.get(DcMotorEx.class, "leftFront");
        esquerdaTras = hardwareMap.get(DcMotorEx.class, "leftBack");
        odometriaX = hardwareMap.get(DcMotorEx.class, "perp");
        odometriaY = hardwareMap.get(DcMotorEx.class, "par");
        RevHubOrientationOnRobot revOrientaion = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revOrientaion));
        direitaFrente.setDirection(DcMotorSimple.Direction.REVERSE);
        direitaTras.setDirection(DcMotorSimple.Direction.FORWARD);
        esquerdaTras.setDirection(DcMotorSimple.Direction.FORWARD);
        esquerdaFrente.setDirection(DcMotorSimple.Direction.FORWARD);
        odometriaY.setDirection(DcMotorSimple.Direction.REVERSE);
        odometriaX.setDirection(DcMotorSimple.Direction.REVERSE);
        imu.resetYaw();
        Thread telemetria = new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
/////////////////////////////////////////////////////Telemetria/////////////////////////////////////////////////////////////////////
                    telemetry.addData("giro", gyro());
                    telemetry.addData("direcao:", direcao);
                    telemetry.addData("controle y:", gamepad1.left_stick_y);
                    telemetry.addData("controle x:", gamepad1.left_stick_x);
                    telemetry.addData("power:", power);
                    telemetry.addData("index:", index);
                    telemetry.addData("Direção", dir);
                    telemetry.addData("concatenado", direcaomovimento);
                    telemetry.addData("giro absoluto:", giroabsoluto);
                    telemetry.addData("posicao abosluta:", direcaoabsoluta);
                    telemetry.addData("modolento", modolento);
                    telemetry.addData("posicao atual:", posicaoatualgarra);
                    telemetry.addData("proxima posicao", proximaposicaogarra);
                    telemetry.addData("state", state);
                    telemetry.addData("odometriaX:", odometriaX.getCurrentPosition());
                    telemetry.addData("odometriaY:", odometriaY.getCurrentPosition());
                    telemetry.update();

                }
            }
        });
        resetarEncoderOdometriasXY();
        waitForStart();
        imu.resetYaw();
        giroabsoluto = 0;
        int movimento;
        int rotacao = 0;

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
                dir = "Frente";
                if (state.equals("parado")) {
                    imu.resetYaw();
                }
                //numdir = 1;
                erro = alvo - gyro();
                proporcional = erro * kp;
                derivativa = (erro - ultimoerro) * kd;
                integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                movDirecionado(Math.abs(gamepad1.left_stick_y) * multiplicador, direcao);
                ultimoerro = erro;
                state = "andando";
                /*movetime = System.currentTimeMillis();
                numerodeciclos = 0;*/
            }

            if (t > f && t > d && t > e) { //andar para trás
                if (state.equals("parado")) {
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
                state = "andando";
                /*movetime = System.currentTimeMillis();
                numerodeciclos = 0;*/
            }

            if (d > f && d > t && d > e) { //andar para Direita
                if (state.equals("parado")) {
                    imu.resetYaw();
                }
                erro = alvo - gyro() ;
                proporcional = erro * 1;
                derivativa = (erro - ultimoerro) * 1;
                integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                mecanumdireitabase(Math.abs(gamepad1.left_stick_x), -direcao);
                state = "andando";
                /*movetime = System.currentTimeMillis();
                numerodeciclos = 0;*/
            }
            if (e > f && e > t && e > d) { //andar para Esquerda
                if (state.equals("parado")) {
                    imu.resetYaw();
                }
                if (state.equals("parado")) {
                    imu.resetYaw();
                }
                erro = alvo - gyro() ;
                proporcional = erro * 1;
                derivativa = (erro - ultimoerro) * 1;
                integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                mecanumesquerdabase(Math.abs(gamepad1.left_stick_x), direcao);
                state = "andando";
                /*movetime = System.currentTimeMillis();
                numerodeciclos = 0;*/
            }
            if (f == 0 && t == 0 && d == 0 && e == 0) {
                parar();
                state = "parado";
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
                    state = "parado";
                }
            }
            if(gamepad1.square){
                if(gyro() > 0){
                    double a = Math.abs((0.2-0.01)/gyro());
                    double b = 0.2;
                    while (gyro() > 0){
                        direitaFrente.setPower((a*gyro()+b) * -1);
                        direitaTras.setPower((a*gyro()+b) * -1);
                        esquerdaFrente.setPower((a*gyro()+b) );
                        esquerdaTras.setPower((a*gyro()+b) );
                    }
                    parar();
                }else{
                    while (gyro() < 0){
                        double a = Math.abs((0.2-0.01)/Math.abs(gyro()));
                        double b = 0.2;
                        direitaFrente.setPower((a*Math.abs(gyro())+b));
                        direitaTras.setPower((a*Math.abs(gyro())+b));
                        esquerdaFrente.setPower((a*Math.abs(gyro())+b)*-1 );
                        esquerdaTras.setPower((a*Math.abs(gyro())+b)*-1);
                    }
                    parar();
                }
                if(odometriaY.getCurrentPosition()>0){
                    double a = (0.2-0.01)/Math.abs(odometriaY.getCurrentPosition());
                    double b = 0.2;
                    while (odometriaY.getCurrentPosition() > 0){
                        erro = alvo - gyro();
                        proporcional = erro * kp;
                        derivativa = (erro - ultimoerro) * kd;
                        integral = erro + ki;
                        direcao = (proporcional + derivativa + integral) / 100;
                        lastpower = gamepad1.left_stick_y * multiplicador;
                        movDirecionado((a*Math.abs(odometriaY.getCurrentPosition())+b)*-1, direcao);
                        ultimoerro = erro;
                        state = "andando";

                    }
                    parar();
                }else{
                    while (odometriaY.getCurrentPosition() < 0){
                        double a = (0.2-0.01)/Math.abs(odometriaY.getCurrentPosition());
                        double b = 0.2;
                        while (odometriaY.getCurrentPosition() > 0){
                            erro = alvo - gyro();
                            proporcional = erro * kp;
                            derivativa = (erro - ultimoerro) * kd;
                            integral = erro + ki;
                            direcao = (proporcional + derivativa + integral) / 100;
                            lastpower = gamepad1.left_stick_y * multiplicador;
                            movDirecionado((a*Math.abs(odometriaY.getCurrentPosition())+b), direcao);
                            ultimoerro = erro;
                            state = "andando";

                        }

                    }
                }
                if(odometriaX.getCurrentPosition() > 0){
                    while (odometriaX.getCurrentPosition() > 0){
                        mecanumDirecionado(0.4, 0);
                        state = "andando";
                    }
                    parar();
                }else{
                    while (odometriaX.getCurrentPosition() < 0){
                        mecanumDirecionado(-0.4, 0);
                        state = "andando";
                    }
                    parar();
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
    public void norteAbosluto(){
        if (giroabsoluto >= 315 || giroabsoluto < 45){//frente
            direcaoabsoluta = 1;
        } else if (giroabsoluto >= 45 && giroabsoluto < 135) {//esquerda
            direcaoabsoluta = 2;
        }else if(giroabsoluto >= 135 && giroabsoluto < 225){//tras
            direcaoabsoluta = 3;
        } else if (giroabsoluto >= 225 && giroabsoluto < 315) {//direita
            direcaoabsoluta = 4;
        }
    }
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

    public void atualizargiro(){
        if (giroabsoluto < 0){
            giroabsoluto = 360 + giroabsoluto;
        }else if (giroabsoluto > 360){
            giroabsoluto = giroabsoluto - 360;
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
    public void resetarimu(){
        giroabsoluto += gyro();
        if (giroabsoluto < 0){
            giroabsoluto = 360 + giroabsoluto;
        }else if (giroabsoluto > 360){
            giroabsoluto = giroabsoluto - 360;
        }
        imu.resetYaw();
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
    public void resetarEncoderOdometriaX(){
        odometriaX.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odometriaX.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public void resetarEncoderOdometriaY(){
        odometriaY.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odometriaY.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public void resetarEncoderOdometriasXY(){
        resetarEncoderOdometriaY();
        resetarEncoderOdometriaX();
    }
    public void mecanumDirecionado(double power, double direcao){
        if (power > 0) {
            if (direcao >= 0){
                direitaFrente.setPower(power*-1);
                direitaTras.setPower(power-direcao);
                esquerdaFrente.setPower(power-direcao);
                esquerdaTras.setPower(power*-1);
            }else{
                direitaFrente.setPower((power+direcao)*-1);
                direitaTras.setPower(power);
                esquerdaFrente.setPower(power);
                esquerdaTras.setPower((power+direcao)*-1);
            }
        }else{
            if (direcao >= 0){
                direitaFrente.setPower((power+direcao)*-1);
                direitaTras.setPower(power);
                esquerdaFrente.setPower(power);
                esquerdaTras.setPower((power+direcao)*-1);
            }else{
                direitaFrente.setPower(power*-1);
                direitaTras.setPower(power-direcao);
                esquerdaFrente.setPower(power-direcao);
                esquerdaTras.setPower(power*-1);
            }
        }
    }

    public double gyro(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}