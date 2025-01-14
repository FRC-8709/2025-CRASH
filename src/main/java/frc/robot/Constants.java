package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

public class Constants {

    public static RobotConfig robotConfig;

    public static void initialize() {
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
