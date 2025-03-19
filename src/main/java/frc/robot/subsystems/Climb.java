package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private final TalonFX climbMotor;
    public final TalonFX reciverMotor;
    private double commandedPosition = 0;

    public Climb(TalonFX climbMotor, TalonFX reciverMotor) {
        this.climbMotor = climbMotor;
        this.reciverMotor = reciverMotor;

        climbMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        reciverMotor.setControl(new PositionDutyCycle(commandedPosition).withSlot(0));
    }

    public void setSpeed(double speed) {
        climbMotor.set(speed);
    }

    public void setCommandedPosition(double position) {
        commandedPosition = position;
    }

    public double currentPosition() {
        return commandedPosition;
    }
}
