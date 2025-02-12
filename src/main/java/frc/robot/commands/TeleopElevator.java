package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class TeleopElevator extends Command {

    private double speed = 0;
    private final Elevator m_subsystem;
    private final Joystick leftOpJoystick;
    private final DigitalInput bottomLimitSwitch;

    public TeleopElevator(Elevator m_subsystem, Joystick leftOpJoystick, DigitalInput bottomLimitSwitch) {
    this.m_subsystem = m_subsystem;
    this.leftOpJoystick = leftOpJoystick;
    this.bottomLimitSwitch = bottomLimitSwitch;

    addRequirements(m_subsystem);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("bottomLimitSwitch", bottomLimitSwitch.get());

        double elevatorPosition = Math.abs(m_subsystem.getElevatorPosition().getValueAsDouble());

        SmartDashboard.putNumber("Elevator Position", elevatorPosition);

        if (!bottomLimitSwitch.get()) {
            m_subsystem.resetElevatorPosition();
        }

        if ( leftOpJoystick.getRawButton(5)) {
            speed = -12;
        }
        else if ( leftOpJoystick.getRawButton(3) && bottomLimitSwitch.get()) {
            speed = 6;
        }
        else if (leftOpJoystick.getRawButton(4) && bottomLimitSwitch.get()) {
            speed = 3;
        }
        else if (leftOpJoystick.getRawButton(6)) {
            speed = -6;
        }
        else if (leftOpJoystick.getRawButton(12)) {
            m_subsystem.resetElevatorPosition();
        }
        else {
            speed = 0;
        }

        if (leftOpJoystick.getRawButton(7) && elevatorPosition < 51.25) {
            if ((51.25 - elevatorPosition) < 6.0 && (51.25 - elevatorPosition) > 1.0) {
                speed = (51.25 - elevatorPosition) * -1;
            } else {
                speed = -6;
            }
        }

        if (leftOpJoystick.getRawButton(9) && elevatorPosition < 81.25) {
            if ((81.25 - elevatorPosition) < 6.0 && (81.25 - elevatorPosition) > 1.0) {
                speed = (81.25 - elevatorPosition) * -1;
            } else {
                speed = -6;
            }
        }

        if (leftOpJoystick.getRawButton(11) && elevatorPosition < 132.5) {
            if ((132.55 - elevatorPosition) < 6.0 && (132.5 - elevatorPosition) > 1.0) {
                speed = (132.5 - elevatorPosition) * -1;
            } else {
                speed = -6;
            }
        }

        m_subsystem.setMotorSpeed(speed);
}


}
