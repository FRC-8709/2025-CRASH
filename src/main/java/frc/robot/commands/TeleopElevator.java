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
    private final DigitalInput bottomLimitSwitch;

    private double commandedPosition = 0;
    private boolean x = false;

    public TeleopElevator(Elevator m_subsystem, Joystick leftOpJoystick, DigitalInput bottomLimitSwitch) {
        this.m_subsystem = m_subsystem;
        this.leftOpJoystick = leftOpJoystick;
        this.bottomLimitSwitch = bottomLimitSwitch;

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

        double elevatorPosition = m_subsystem.getElevatorPosition().getValueAsDouble();

        if (leftOpJoystick.getRawButtonPressed(7)) commandedPosition = -138.5;
        if (leftOpJoystick.getRawButtonPressed(9)) commandedPosition = -89.5;
        if (leftOpJoystick.getRawButtonPressed(11)) commandedPosition = -56;
        if (leftOpJoystick.getRawButtonPressed(6)) commandedPosition = elevatorPosition - 5;
        if (leftOpJoystick.getRawButtonPressed(12) && bottomLimitSwitch.get()) commandedPosition = elevatorPosition + 0.3;

        SmartDashboard.putBoolean("bottomLimitSwitch", bottomLimitSwitch.get());
        SmartDashboard.putNumber("Elevator Position", elevatorPosition);
        SmartDashboard.putNumber("Commanded Position", commandedPosition);

        if (!bottomLimitSwitch.get()) {
            m_subsystem.resetElevatorPosition();
            elevatorPosition = 0.0;
        }
        if (!bottomLimitSwitch.get() && !x) {
            commandedPosition = 0.0;
            x = true;
        }
        if (bottomLimitSwitch.get()) {
            x = false;
        }

        m_subsystem.masterMotor.setControl(new PositionDutyCycle(commandedPosition).withSlot(0));
        m_subsystem.followerMotor.setControl(new PositionDutyCycle(-commandedPosition).withSlot(0));
        if (elevatorPosition > 90) RobotContainer.MaxSpeed = 0.75;
        else if (elevatorPosition > 60) RobotContainer.MaxSpeed = 1.5;
        else if (elevatorPosition > 15) RobotContainer.MaxSpeed = 2.96;
        else RobotContainer.MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        SmartDashboard.putNumber("Max Speed", RobotContainer.MaxSpeed);
}


}
