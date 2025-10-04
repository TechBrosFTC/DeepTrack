package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.deeptrack.Armazenamento;
public class Seletor {
    Armazenamento armazenamento;
    DcMotorEx encoder;
    Servo servo;
    int posicaoatual = 0;
    public Seletor(Armazenamento armazenamento, DcMotorEx encoder, Servo servo){
        this.armazenamento = armazenamento;
        this.encoder = encoder;
        this.servo = servo;
    }
    public void posicaocoleta(int posicaoalvo){
        if(posicaoalvo - posicaoatual > 0){
            posicaoatual = 0;
        }
        posicaoatual = posicaoalvo;
    }
    public void posiçaolancamento(){
    }
}
