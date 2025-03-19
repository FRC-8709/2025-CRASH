package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private enum DriveState {
        JOYSTICK_CONTROL,
        ALIGN_LEFT,
        ALIGN_RIGHT
    }

    private double rightTagID;
    private double leftTagID;

    private SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private final Limelight s_Limelight;
    private final AutoAlignment s_AutoAlignment;
    private final Joystick rightDriveJoystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive;
    private final Joystick leftDriveJoystick;
    private final double MaxSpeed;
    private final double MaxAngularRate;
    DriveState teleopControl = DriveState.JOYSTICK_CONTROL;

    public Drive(CommandSwerveDrivetrain drivetrain, Joystick leftDriveJoystick, double MaxSpeed, double MaxAngularRate, SwerveRequest.FieldCentric drive, Joystick rightDriveJoystick, Limelight s_Limelight, AutoAlignment s_AutoAlignment) {
        this.drive = drive;
        this.drivetrain = drivetrain;
        this.leftDriveJoystick = leftDriveJoystick;
        this.MaxSpeed = MaxSpeed;
        this.MaxAngularRate = MaxAngularRate;
        this.rightDriveJoystick = rightDriveJoystick;
        this.s_Limelight = s_Limelight;
        this.s_AutoAlignment = s_AutoAlignment;
    }

    @Override
    public void periodic() {
        rightTagID = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("tid").getDouble(0);
        leftTagID = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tid").getDouble(0);
        SmartDashboard.putNumber("Right Tag ID", rightTagID);

        switch(teleopControl) {
            case JOYSTICK_CONTROL:
                drivetrain.setControl(drive.withVelocityX(-leftDriveJoystick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-leftDriveJoystick.getX() * MaxSpeed).withDeadband(1) // Drive left with negative X (left)
                        .withRotationalRate(-rightDriveJoystick.getX() * MaxAngularRate).withRotationalDeadband(1));
                break;
                case ALIGN_LEFT:
                    drivetrain.setControl(robotCentric.withVelocityY(s_AutoAlignment.leftAlignSpeed * MaxSpeed) // Drive forward with negative Y (forward)
                            .withVelocityX(0.0) // Drive left with negative X (left)
                            .withRotationalRate(0.0));
                    break;
                    case ALIGN_RIGHT:
                        drivetrain.setControl(robotCentric.withVelocityY(s_AutoAlignment.rightAlignSpeed * MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityX(0.0) // Drive left with negative X (left)
                                .withRotationalRate(0.0));
        }

        if( leftDriveJoystick.getRawButton(1)) {
            teleopControl = DriveState.ALIGN_LEFT;
        }
        else if (rightDriveJoystick.getRawButton(1)) {
            teleopControl = DriveState.ALIGN_RIGHT;
        }
        else {
            teleopControl = DriveState.JOYSTICK_CONTROL;
        }
    }
}

