package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

public class TeleopCoralIntake extends Command {
    private double speed = 0;
    private final CoralIntake m_subsystem;
    private final Joystick rightOpJoystick;

    public TeleopCoralIntake (CoralIntake m_subsystem, Joystick rightOpJoystick) {
        this.m_subsystem = m_subsystem;
        this.rightOpJoystick = rightOpJoystick;
        addRequirements(m_subsystem);
    }

    @Override
    public void execute() {
        if ( rightOpJoystick.getRawButton(5)) {
            speed = 4;
        }
        else if ( rightOpJoystick.getRawButton(3)) {
            speed = 2;
        }
        else if ( rightOpJoystick.getRawButton(4)) {
            speed = 1;
        }
        else if (rightOpJoystick.getRawButton(6)) {
            speed = -1;
        }
        else {
            speed = 0;
        }
        m_subsystem.setMotorSpeed(speed);
    }
}
