package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    public final TalonFX masterMotor;
    public final TalonFX followerMotor;

    public Elevator(TalonFX masterMotor, TalonFX followerMotor) {
        this.masterMotor = masterMotor;
        this.followerMotor = followerMotor;

        masterMotor.setNeutralMode(NeutralModeValue.Brake);
        followerMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    public void setMotorSpeed(double speed) { 
        masterMotor.setControl(ElevatorConstants.kElevatorVoltageOut.withOutput(speed));
        followerMotor.setControl(ElevatorConstants.kElevatorVoltageOut.withOutput(-speed));
    }
}
