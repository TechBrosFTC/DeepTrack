package org.firstinspires.ftc.teamcode.deeptrack;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * This is the class for the IMU sensor. This class is used on the robot setup.
 */

public class InertialMeasurementUnit {
    public RevHubOrientationOnRobot orientation;
    public IMU imu;
    public InertialMeasurementUnit(RevHubOrientationOnRobot orientation, HardwareMap hardwareMap, String imuName){
        this.orientation = orientation;
        this.imu = hardwareMap.get(IMU.class, imuName);
        this.imu.initialize(new IMU.Parameters(orientation));
    }
    public void resetYaw(){
        this.imu.resetYaw();
    }
    public double getYaw() {
        return this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
