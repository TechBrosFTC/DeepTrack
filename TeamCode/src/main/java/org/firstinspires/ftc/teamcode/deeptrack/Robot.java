package org.firstinspires.ftc.teamcode.deeptrack;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**



 */
public class Robot {
    public  TwoWheelOdometry odometry;
    public Robot(HardwareMap hardwareMap) {
        odometry = new TwoWheelOdometry(hardwareMap,false, false);
    }

}
