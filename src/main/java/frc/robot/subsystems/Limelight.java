package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
    private boolean rightIsCentered = false;
    private boolean leftIsCentered = false;
    private double txRight;
    private double txLeft;
    private double rightTagID;
    private double leftTagID;

    @Override
    public void periodic() {
        rightTagID = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("tid").getDouble(0);
        leftTagID = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tid").getDouble(0);
        txRight = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("tx").getDouble(-100.0);
        txLeft = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tx").getDouble(-100.0);
        SmartDashboard.putNumber("txRight", txRight);
        SmartDashboard.putNumber("txLeft", txLeft);
        SmartDashboard.putNumberArray("botPose", LimelightHelpers.getBotPose("limelight-right"));

        if (txRight > 0 && rightTagID != -1) {
            rightIsCentered = true;
        } else {
            rightIsCentered = false;
        }
        if (txLeft < -3.1 && leftTagID != -1) {
            leftIsCentered = true;
        } else {
            leftIsCentered = false;
        }
        SmartDashboard.putBoolean("isCentered", rightCentered());
    }

    public boolean rightCentered() {
        return rightIsCentered;
    }

    public boolean leftCentered() {
        return leftIsCentered;
    }

    public double getRightX() {
        return txRight;
    }

    public double getLeftX() {
        return txLeft;
    }
}

