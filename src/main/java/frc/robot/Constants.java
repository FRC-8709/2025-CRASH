package frc.robot;

import com.ctre.phoenix6.controls.VoltageOut;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

public class Constants {

    public static RobotConfig robotConfig;

    public static void initialize() {
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    public class CoralIntakeConstants {
        public static int beamBreakPort = 9;
        public static int coralIntakeMotorPort = 24;
        public static final VoltageOut kCoralIntakeVoltageOut = new VoltageOut(0);
    }
    public class AlgaeIntakeConstants {
        public static int algaeIntakeMotorPort = 23;
        public static final VoltageOut kAlgaeIntakeVoltageOut = new VoltageOut(0);
    }
    public class PathPlannerConstants {
        public static final PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    }

    public static final class ElevatorConstants {
        public static final VoltageOut kElevatorVoltageOut = new VoltageOut(0);

        public static final int bottomLimitSwitchPort = 8;

        public static final int leftElevatorMotorPort = 28;
        public static final int rightElevatorMotorPort = 18;
    }

    public static final class ClimberConstants {
        public static final int climberMotorPort = 19;
        public static final int reciverMotorPort = 30;
    }
}
