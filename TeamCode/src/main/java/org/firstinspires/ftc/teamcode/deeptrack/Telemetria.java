package org.firstinspires.ftc.teamcode.deeptrack;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous (name = "telemtria")
public class Telemetria extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        robot.odometry.resetOdometryEncoder();
        while (opModeIsActive()) {
            telemetry.addData("Get Y:", robot.odometry.getY());
            telemetry.addData("Get X: ", robot.odometry.getX());
            telemetry.update();
        }
    }
}
