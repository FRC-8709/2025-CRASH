package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class TeleopClimb extends Command {
    private final Joystick leftOpJoystick;
    private final Climb m_subsystem;
    private double speed;
    public TeleopClimb(Joystick leftOpJoystick, Climb subsystem) {
        this.leftOpJoystick = leftOpJoystick;
        this.m_subsystem = subsystem;

        addRequirements(m_subsystem);
    }

    @Override
    public void execute() {
        if (leftOpJoystick.getRawButton(3)) {
            speed = 3.0;
        }
        else if (leftOpJoystick.getRawButton(5)) {
            speed = -3.0;
        }
        else {
            speed = 0.0;
        }
        m_subsystem.setSpeed(speed);
    }
}
