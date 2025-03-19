package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoAlignment extends SubsystemBase {
    private static double kDt = 0.02;
    private static double kMaxVelocity = 1.75;
    private static double kMaxAcceleration = 0.75;
    private static double kP = 0.007;
    private static double kI = 0.0;
    private static double kD = 0;
    private static double kS = 0.0;
    private static double kG = 0.0;
    private static double kV = 0.0;
    public double leftAlignSpeed = 0.0;
    public double rightAlignSpeed = 0.0;

    private final Joystick m_joystick = new Joystick(1);
    private final Limelight s_Limelight;

    // Create a PID controller whose setpoint's change is subject to maximum
    // velocity and acceleration constraints.
    private final TrapezoidProfile.Constraints m_constraints =
            new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
    private final ProfiledPIDController m_leftController =
            new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
    private final ProfiledPIDController m_rightController =
            new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);

    public AutoAlignment(Limelight s_Limelight) {
        this.s_Limelight = s_Limelight;
    }

    @Override
    public void periodic() {
        m_leftController.setGoal(0);
        m_rightController.setGoal(0);

        // Run controller and update motor output
        leftAlignSpeed = m_leftController.calculate(s_Limelight.getRightX())
                + m_feedforward.calculate(m_leftController.getSetpoint().velocity);
        rightAlignSpeed = m_rightController.calculate(s_Limelight.getLeftX())
                + m_feedforward.calculate(m_rightController.getSetpoint().velocity);
        SmartDashboard.putNumber("leftAlignSpeed", leftAlignSpeed);
        SmartDashboard.putNumber("rightAlignSpeed", rightAlignSpeed);
    }
}