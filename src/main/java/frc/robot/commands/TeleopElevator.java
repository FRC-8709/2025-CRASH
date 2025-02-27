package frc.robot.commands;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class TeleopElevator extends Command {

    //private double speed = 0;
    private final Elevator m_subsystem;
    private final Joystick leftOpJoystick;
    private final DigitalInput bottomLimitSwitch;

    private double commandedPosition = 0;

    public TeleopElevator(Elevator m_subsystem, Joystick leftOpJoystick, DigitalInput bottomLimitSwitch) {
        this.m_subsystem = m_subsystem;
        this.leftOpJoystick = leftOpJoystick;
        this.bottomLimitSwitch = bottomLimitSwitch;

        addRequirements(m_subsystem);
        Slot0Configs configs = new Slot0Configs();
        configs.withKP(0.05);
        configs.withKI(0);
        configs.withKD(0.02);
        m_subsystem.masterMotor.getConfigurator().apply(configs);
    }

    @Override
    public void execute() {

        double elevatorPosition = m_subsystem.getElevatorPosition().getValueAsDouble();

        if (leftOpJoystick.getRawButtonPressed(7)) commandedPosition = -100;
        if (leftOpJoystick.getRawButtonPressed(9)) commandedPosition = -67;
        if (leftOpJoystick.getRawButtonPressed(11)) commandedPosition = -35.5;
        if (leftOpJoystick.getRawButtonPressed(12)) commandedPosition = 0.0;
        if (leftOpJoystick.getRawButtonPressed(6)) commandedPosition = elevatorPosition - 5;
        else if (leftOpJoystick.getRawButtonPressed(4) && elevatorPosition < -5.0) commandedPosition = elevatorPosition + 5;

        SmartDashboard.putBoolean("bottomLimitSwitch", bottomLimitSwitch.get());


        SmartDashboard.putNumber("Elevator Position", elevatorPosition);

        if (!bottomLimitSwitch.get()) {
            m_subsystem.resetElevatorPosition();
            m_subsystem.masterMotor.setPosition(0);
        }

        m_subsystem.masterMotor.setControl(new PositionDutyCycle(commandedPosition).withSlot(0));
        if (elevatorPosition > 90) RobotContainer.MaxSpeed = 0.75;
        else if (elevatorPosition > 60) RobotContainer.MaxSpeed = 1.5;
        else if (elevatorPosition > 15) RobotContainer.MaxSpeed = 2.96;
        else RobotContainer.MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        SmartDashboard.putNumber("Max Speed", RobotContainer.MaxSpeed);
}


}
