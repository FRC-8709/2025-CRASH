package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Limelight extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive;

    double rotSpeed = 0.0;
    boolean centered = false;

    public Limelight(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive) {
        this.drivetrain = drivetrain;
        this.drive = drive;
    }

    @Override
    public void periodic() {
        double txRight = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("tx").getDouble(0);
        double txLeft = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tx").getDouble(0);
        SmartDashboard.putNumber("txRight", txRight); //+9
        SmartDashboard.putNumber("txLeft", txLeft); //-9
        if (txRight < 9 && txLeft > -9) {
            centered = false;
            rotSpeed = 1.0;
//            drivetrain.setControl(
//                    drive.withVelocityX(0.0 * 1) // Drive forward with negative Y (forward)
//                            .withVelocityY(0.0 * 1) // Drive left with negative X (left)
//                            .withRotationalRate(-rotSpeed * 1) // Drive counterclockwise with negative X (left)
//            );
        }
        else {
            centered = true;
            rotSpeed = 0.0;
        }
        SmartDashboard.putBoolean("in center", centered);
    }
    public boolean isCentered() {
        return centered;
    }
}
