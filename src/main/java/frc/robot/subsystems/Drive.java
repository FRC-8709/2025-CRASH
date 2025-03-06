package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        DriveState teleopControl;
        rightTagID = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("tid").getDouble(0);
        leftTagID = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tid").getDouble(0);
        SmartDashboard.putNumber("Right Tag ID", rightTagID);


        if (leftDriveJoystick.getRawButton(1) && (s_Limelight.getRightX() < 0.0 || rightTagID == -1)) {
            teleopControl = DriveState.ALIGN_LEFT;
        } else if (rightDriveJoystick.getRawButton(1) && !s_Limelight.leftCentered()) {
            teleopControl = DriveState.ALIGN_RIGHT;
        } else {
            teleopControl = DriveState.JOYSTICK_CONTROL;
        }
        switch (teleopControl) {
            case JOYSTICK_CONTROL:
                drivetrain.setControl(drive.withVelocityX(-leftDriveJoystick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-leftDriveJoystick.getX() * MaxSpeed).withDeadband(1) // Drive left with negative X (left)
                        .withRotationalRate(-rightDriveJoystick.getX() * MaxAngularRate).withRotationalDeadband(1));
                break;
            case ALIGN_LEFT:
                if (rightTagID == -1) {
                    drivetrain.setControl(robotCentric
                            .withVelocityY(0.3)
                            .withVelocityX(0.0)
                            .withRotationalRate(0.0)
                    );
                } else if (s_Limelight.getRightX() < -11 && s_Limelight.getRightX() > -20) {
                    drivetrain.setControl(robotCentric
                            .withVelocityY(s_Limelight.getRightX() / -40)
                            .withVelocityX(0.15)
                            .withRotationalRate(0.0)
                    );
                } else if (s_Limelight.getRightX() > -11) {
                    drivetrain.setControl(robotCentric
                            .withVelocityY(0.15)
                            .withVelocityX(0.15)
                            .withRotationalRate(0.0)
                    );
                } else if (s_Limelight.getRightX() > 5) {
                    drivetrain.setControl(robotCentric
                            .withVelocityY(-0.15)
                            .withVelocityX(0.15)
                            .withRotationalRate(0.0)
                    );
                } else {
                    drivetrain.setControl(robotCentric
                            .withVelocityY(0.0)
                            .withVelocityX(0.0)
                            .withRotationalRate(0.0)
                    );
                }
                break;
            case ALIGN_RIGHT:
                if (leftTagID == -1) {
                    drivetrain.setControl(robotCentric
                            .withVelocityY(-0.3)
                            .withVelocityX(0.0)
                            .withRotationalRate(0.0)
                    );
                } else if (s_Limelight.getLeftX() > 11 && s_Limelight.getLeftX() < 20) {
                    drivetrain.setControl(robotCentric
                            .withVelocityY(s_Limelight.getLeftX() / -40)
                            .withVelocityX(0.15)
                            .withRotationalRate(0.0)
                    );
                } else if (s_Limelight.getLeftX() < 11) {
                    drivetrain.setControl(robotCentric
                            .withVelocityY(-0.15)
                            .withVelocityX(0.15)
                            .withRotationalRate(0.0)
                    );
                } else if (s_Limelight.getLeftX() < -8.1) {
                    drivetrain.setControl(robotCentric
                            .withVelocityY(0.15)
                            .withVelocityX(0.15)
                            .withRotationalRate(0.0)
                    );
                } else {
                    drivetrain.setControl(robotCentric
                            .withVelocityY(0.0)
                            .withVelocityX(0.0)
                            .withRotationalRate(0.0)
                    );
                }
        }
    }
}
