package org.firstinspires.ftc.teamcode.deeptrack;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RobotSetup;
@Autonomous (name = "telemtria")
public class Telemetria extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotSetup robot = new RobotSetup(hardwareMap);
        while (opModeIsActive()) {
        }
    }
}
