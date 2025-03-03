package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
    private boolean isCentered = false;

    @Override
    public void periodic() {
        double txRight = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("tx").getDouble(0.0);
        double txLeft = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tx").getDouble(0.0);
        SmartDashboard.putNumber("txRight", txRight);
        SmartDashboard.putNumber("txLeft", txLeft);
        if (txRight < -6 && txLeft > 11) {
            isCentered = true;
        }
        else {
            isCentered = false;
        }
        SmartDashboard.putBoolean("isCentered", centered());
    }
    public boolean centered() {
        return isCentered;
    }
}

