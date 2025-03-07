package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private final TalonFX climbMotor;

    public Climb(TalonFX climbMotor) {
        this.climbMotor = climbMotor;

        climbMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setSpeed(double speed) {
        climbMotor.set(speed);
    }
}
