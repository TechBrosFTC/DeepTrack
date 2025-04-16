package org.firstinspires.ftc.teamcode.deeptrack;

/**



 */
public class Robot {
    public static TwoWheelOdometry odometry;
    public static void robotInit() {
        odometry = new TwoWheelOdometry("perp", "par", false, false);


    }
}
