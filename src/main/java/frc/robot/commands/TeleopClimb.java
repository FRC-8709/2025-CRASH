package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class TeleopClimb extends Command {
    private final Joystick buttonBox;
    private final Climb m_subsystem;
    private double speed;

    public TeleopClimb(Joystick buttonBox, Climb subsystem) {
        this.buttonBox = buttonBox;
        this.m_subsystem = subsystem;

        addRequirements(m_subsystem);
    }

    @Override
    public void execute() {
        if (buttonBox.getRawButton(18)) {
            speed = 3.0;
        } else if (buttonBox.getRawButton(19)) {
            speed = -3.0;
        } else {
            speed = 0.0;
        }
        m_subsystem.setSpeed(speed);
    }
}
