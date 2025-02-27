package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        followerMotor.setControl(new Follower(masterMotor.getDeviceID(), true));
    }

    @Override
    public void periodic() {
        double elevatorPosition = Math.abs(getElevatorPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Position", elevatorPosition);
    }

    public void setMotorSpeed(double speed) {
        masterMotor.setControl(ElevatorConstants.kElevatorVoltageOut.withOutput(speed));
        followerMotor.setControl(ElevatorConstants.kElevatorVoltageOut.withOutput(-speed));
    }
    public StatusSignal<Angle> getElevatorPosition() {
        return masterMotor.getPosition();
    }
    public void resetElevatorPosition() {
        masterMotor.setPosition(0.0f);
    }
}