package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

    private final SwerveDrivetrain drivetrain;
    private final Pose2d middleRightCoralPoseBlue = new Pose2d(6.198, 3.941, Rotation2d.fromDegrees(180.0));
    public Pose2d targetPose;


    public Limelight(SwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public Command middleRightCoralPathBlue(Pose2d targetPose) {
        return AutoBuilder.pathfindToPose(targetPose, Constants.PathPlannerConstants.constraints);
    }

}
