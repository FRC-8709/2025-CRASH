package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
    private final TalonFX intakeMotor;
    public AlgaeIntake(TalonFX intakeMotor) {
        this.intakeMotor = intakeMotor;

        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    public void setMotorSpeed(double speed) {
        intakeMotor.setControl(Constants.AlgaeIntakeConstants.kAlgaeIntakeVoltageOut.withOutput(speed));
    }
    public Command intakeAlgae() {
        return this.runOnce(() -> {
            try {
                setMotorSpeed(2);
                wait(500);
                setMotorSpeed(0);
            }
            catch (Exception ignored) {}
        });
    }
}
