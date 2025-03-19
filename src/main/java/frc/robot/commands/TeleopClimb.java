package frc.robot.commands;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class TeleopClimb extends Command {
    private final Joystick rightDriveJoystick;
    private final Climb m_subsystem;
    private double speed;

    public TeleopClimb(Joystick rightDriveJoystick, Climb subsystem) {
        this.rightDriveJoystick = rightDriveJoystick;
        this.m_subsystem = subsystem;

        addRequirements(m_subsystem);
        Slot0Configs configs = new Slot0Configs();
        configs.withKP(0.1);
        configs.withKI(0.00);
        configs.withKD(0.02);
        m_subsystem.reciverMotor.getConfigurator().apply(configs);
    }

    @Override
    public void execute() {
        if (rightDriveJoystick.getRawButton(11)) {
            speed = 3.0;
        } else if (rightDriveJoystick.getRawButton(12)) {
            speed = -3.0;
        } else if (rightDriveJoystick.getRawButton(9)) {
            m_subsystem.setCommandedPosition(-20);
        } else if (rightDriveJoystick.getRawButton(10)) {
            m_subsystem.setCommandedPosition(20);
        } else if (rightDriveJoystick.getRawButton(7)) {
            m_subsystem.setCommandedPosition(m_subsystem.currentPosition() - 1);
        } else if (rightDriveJoystick.getRawButton(8)) {
            m_subsystem.setCommandedPosition(m_subsystem.currentPosition() + 1);
        } else {
            speed = 0.0;
        }
        m_subsystem.setSpeed(speed);
    }
}
