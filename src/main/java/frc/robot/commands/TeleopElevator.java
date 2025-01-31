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
    private final Encoder elevatorEncoder;
    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    public TeleopElevator(Elevator m_subsystem, Joystick leftOpJoystick, Encoder elevatorEncoder, DigitalInput topLimitSwitch, DigitalInput bottomLimitSwitch) {
    this.m_subsystem = m_subsystem;
    this.leftOpJoystick = leftOpJoystick;
    this.elevatorEncoder = elevatorEncoder;
    this.topLimitSwitch = topLimitSwitch;
    this.bottomLimitSwitch = bottomLimitSwitch;

    addRequirements(m_subsystem);
    }

    @Override 
    public void execute() { 
        SmartDashboard.putNumber("Elevator Encoder", elevatorEncoder.getDistance());
        SmartDashboard.putBoolean("topLimitSwitch", !topLimitSwitch.get());
        SmartDashboard.putBoolean("bottomLimitSwitch", !bottomLimitSwitch.get());

        if ( leftOpJoystick.getRawButton(5)) {
            speed = 12;
        }
        else if ( leftOpJoystick.getRawButton(3)) {
            speed = -6;
        }
        else if (leftOpJoystick.getRawButton(4) && elevatorEncoder.get() > 0.2) {
            speed = -3;
        }
        else if (leftOpJoystick.getRawButton(6) && elevatorEncoder.get() < 0.8) {
            speed = 3;
        }
        else {
            speed = 0;
        }
        m_subsystem.setMotorSpeed(speed);
}


}
