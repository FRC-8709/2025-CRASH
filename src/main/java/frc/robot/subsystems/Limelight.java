package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
    private final Joystick rightDriveJoystick;
    private final Joystick leftDriveJoystick;
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    public Limelight(Joystick rightDriveJoystick, Joystick leftDriveJoystick) {
        this.rightDriveJoystick = rightDriveJoystick;
        this.leftDriveJoystick = leftDriveJoystick;
    }

    double limelight_aim_proportional() {
        final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
        double kP = 1;

        double targetingAngularVelocity = LimelightHelpers.getTX("limelight-left") * kP;

        targetingAngularVelocity *= 5.96;

        targetingAngularVelocity *= -1.0;


        return targetingAngularVelocity;
    }

    double limelight_range_proportional() {
        double kP = .1;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight-left") * kP;
        targetingForwardSpeed *= 5.96;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }

    public void drive(boolean FieldRelative) {

        var rot =
                -m_rotLimiter.calculate(MathUtil.applyDeadband(rightDriveJoystick.getX(), 0.02))
                        * 5.96;

        if (leftDriveJoystick.getRawButton(10)) {
            final var rot_limelight = limelight_aim_proportional();
            rot = rot_limelight;
        }
    }
}

