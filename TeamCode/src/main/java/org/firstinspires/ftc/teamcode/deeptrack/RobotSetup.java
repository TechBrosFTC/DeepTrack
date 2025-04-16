package org.firstinspires.ftc.teamcode.deeptrack;
import org.firstinspires.ftc.teamcode.deeptrack.TwoWheelOdometry;
/**



 */
public class RobotSetup {
    public void robotInit() {
        TwoWheelOdometry odometry = new TwoWheelOdometry("odometry_x", "odometry_y", false, false);


    }
}
