package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.TeleopElevator;

public class Elevator extends SubsystemBase {

    public final TalonFX masterMotor;
    public final TalonFX followerMotor;

    private double commandedPosition = 0;

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

        masterMotor.setControl(new PositionDutyCycle(commandedPosition).withSlot(0));
        followerMotor.setControl(new PositionDutyCycle(-commandedPosition).withSlot(0));
    }

    public StatusSignal<Angle> getElevatorPosition() {
        return followerMotor.getPosition();
    }

    public void resetElevatorPosition() {
        masterMotor.setPosition(0.0);
        followerMotor.setPosition(0.0);
    }

    public void setCommandedPosition(double position) {
        commandedPosition = position;
    }

    public Command elevatorUp() {
        return this.runOnce(() -> {
            try {
                setCommandedPosition(-136.5);
            } catch (Exception ignored) {
            }
        });
    }

    public Command elevatorDown() {
        return this.runOnce(() -> {
            try {
                setCommandedPosition(0.0);
            } catch (Exception ignored) {}
        });
    }
}