package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

public class TeleopAlgaeIntake extends Command {
    private final Joystick rightOpJoystick;
    private final AlgaeIntake m_subsystem;

    public TeleopAlgaeIntake(Joystick rightOpJoystick, AlgaeIntake m_subsystem) {
        this.rightOpJoystick = rightOpJoystick;
        this.m_subsystem = m_subsystem;

        addRequirements(m_subsystem);
    }

    @Override
    public void execute() {
        if (rightOpJoystick.getRawButton(3)) {
            m_subsystem.setMotorSpeed(-2);
        } else if (rightOpJoystick.getRawButton(4)) {
            m_subsystem.setMotorSpeed(2);
        } else {
            m_subsystem.setMotorSpeed(0);
        }
    }
}
