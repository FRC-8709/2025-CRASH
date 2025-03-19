package frc.robot.commands;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class TeleopElevator extends Command {

    private final Elevator m_subsystem;
    private final Joystick leftOpJoystick;
    private final Joystick rightOpJoystick;
    private final DigitalInput bottomLimitSwitch;
    private double elevatorPosition;

    private boolean x = false;

    public TeleopElevator(Elevator m_subsystem, Joystick leftOpJoystick, DigitalInput bottomLimitSwitch, Joystick rightOpJoystick) {
        this.m_subsystem = m_subsystem;
        this.leftOpJoystick = leftOpJoystick;
        this.bottomLimitSwitch = bottomLimitSwitch;
        this.rightOpJoystick = rightOpJoystick;

        addRequirements(m_subsystem);
        Slot0Configs configs = new Slot0Configs();
        configs.withKP(0.1);
        configs.withKI(0.00);
        configs.withKD(0.02);
        m_subsystem.masterMotor.getConfigurator().apply(configs);
        m_subsystem.followerMotor.getConfigurator().apply(configs);
    }

    @Override
    public void execute() {

        elevatorPosition = m_subsystem.getElevatorPosition().getValueAsDouble();

        if (leftOpJoystick.getRawButtonPressed(7)) m_subsystem.setCommandedPosition(-136.5);
        if (leftOpJoystick.getRawButtonPressed(9)) m_subsystem.setCommandedPosition(-87.5);
        if (leftOpJoystick.getRawButtonPressed(11)) m_subsystem.setCommandedPosition(-54);
        if (leftOpJoystick.getRawButtonPressed(12) && bottomLimitSwitch.get()) m_subsystem.setCommandedPosition(elevatorPosition + 0.3);
        if (rightOpJoystick.getRawButtonPressed(9)) m_subsystem.setCommandedPosition(-48.5);
        if (rightOpJoystick.getRawButtonPressed(11)) m_subsystem.setCommandedPosition(-8);
        if (rightOpJoystick.getRawButtonPressed(12) && bottomLimitSwitch.get()) m_subsystem.setCommandedPosition(elevatorPosition + 0.3);
        SmartDashboard.putBoolean("bottomLimitSwitch", bottomLimitSwitch.get());
        SmartDashboard.putNumber("Elevator Position", elevatorPosition);

        if (!bottomLimitSwitch.get() && elevatorPosition > -20) {
            m_subsystem.resetElevatorPosition();
            elevatorPosition = 0.0;
        }
        if (!bottomLimitSwitch.get() && !x) {
            m_subsystem.setCommandedPosition(0.0);
            x = true;
        }
        if (bottomLimitSwitch.get()) {
            x = false;
        }

                if (elevatorPosition > 90) RobotContainer.MaxSpeed = 0.75;
        else if (elevatorPosition > 60) RobotContainer.MaxSpeed = 1.5;
        else if (elevatorPosition > 15) RobotContainer.MaxSpeed = 2.96;
        else RobotContainer.MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        SmartDashboard.putNumber("Max Speed", RobotContainer.MaxSpeed);
    }
    public double getElevatorPosition() {
        return elevatorPosition;
    }
}
