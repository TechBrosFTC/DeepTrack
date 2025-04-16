package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.deeptrack.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Telemetria extends LinearOpMode {
    @Override


    public void runOpMode() throws InterruptedException {
        Robot.robotInit();
        Robot.odometry.resetOdometryEncoder();
        while (opModeIsActive()) {
            telemetry.addData("Get Y:", Robot.odometry.getY());
            telemetry.addData("Get X: ", Robot.odometry.getX());
            telemetry.update();
        }
    }
}
