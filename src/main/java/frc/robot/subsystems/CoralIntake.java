package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
    public final TalonFX intakeMotor;

    public CoralIntake(TalonFX intakeMotor) {
        this.intakeMotor = intakeMotor;

        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setMotorSpeed(double speed) {
        intakeMotor.setControl(Constants.CoralIntakeConstants.kCoralIntakeVoltageOut.withOutput(speed));
    }

    public Command scoreCoral() {
        return this.runOnce(() -> {
            try {
                setMotorSpeed(4);
                wait(3000);
                setMotorSpeed(0);
            } catch (Exception ignored) {}
        });
    }
}
