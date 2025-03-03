package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private String driveState = "JOYSTICK_CONTROL";
    private SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private final Limelight s_Limelight;
    private final Joystick rightDriveJoystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive;
    private final Joystick leftDriveJoystick;
    private final double MaxSpeed;
    private final double MaxAngularRate;

    public Drive(CommandSwerveDrivetrain drivetrain, Joystick leftDriveJoystick, double MaxSpeed, double MaxAngularRate, SwerveRequest.FieldCentric drive, Joystick rightDriveJoystick, Limelight s_Limelight) {
        this.drive = drive;
        this.drivetrain = drivetrain;
        this.leftDriveJoystick = leftDriveJoystick;
        this.MaxSpeed = MaxSpeed;
        this.MaxAngularRate = MaxAngularRate;
        this.rightDriveJoystick = rightDriveJoystick;
        this.s_Limelight = s_Limelight;
    }

    @Override
    public void periodic() {
        switch(driveState) {
            case "JOYSTICK_CONTROL":
                drivetrain.setControl(drive.withVelocityX(-leftDriveJoystick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-leftDriveJoystick.getX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-rightDriveJoystick.getX() * MaxAngularRate));
                break;
            case "LIMELIGHT_CONTROL":
                drivetrain.setControl(robotCentric.withRotationalRate(-0.1 * 5.96));
        }
        if (rightDriveJoystick.getRawButton(1) && !s_Limelight.centered()) {
            driveState = "LIMELIGHT_CONTROL";
        }
        else {
            driveState = "JOYSTICK_CONTROL";
        }
    }
}
