package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
    private boolean rightIsCentered = false;
    private double txRight;
    private double txLeft;
    private double rightTagID;
    private double leftTagID;

    @Override
    public void periodic() {
        // distance from the target to the floor
        double goalHeightInches = 60.0;

        NetworkTable tableRight = NetworkTableInstance.getDefault().getTable("limelight-right");
        NetworkTableEntry tyRight = tableRight.getEntry("ty");
        double targetOffsetAngle_VerticalRight = tyRight.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegreesRight = 25.0;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInchesRight = 20.0;

        double angleToGoalDegreesRight = limelightMountAngleDegreesRight + targetOffsetAngle_VerticalRight;
        double angleToGoalRadiansRight = angleToGoalDegreesRight * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInchesRight = (goalHeightInches - limelightLensHeightInchesRight) / Math.tan(angleToGoalRadiansRight);

        rightTagID = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("tid").getDouble(0);
        leftTagID = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tid").getDouble(0);
        txRight = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("tx").getDouble(0);
        txLeft = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tx").getDouble(0);
        SmartDashboard.putNumber("txRight", txRight);
        SmartDashboard.putNumber("txLeft", txLeft);
        SmartDashboard.putNumberArray("botPose", LimelightHelpers.getBotPose("limelight-right"));

        if (txRight > 0 && rightTagID != -1) {
            rightIsCentered = true;
        } else {
            rightIsCentered = false;
        }
        SmartDashboard.putBoolean("isCentered", rightIsCentered);
    }

    public double getRightX() {
        if (rightTagID != -1) {
            txRight = 20;
        }
        return txRight;
    }

    public double getLeftX() {
        if (leftTagID != -1) {
            txLeft = -20;
        }
        return txLeft;
    }
}

