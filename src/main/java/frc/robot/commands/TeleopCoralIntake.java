package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

public class TeleopCoralIntake extends Command {
    private double speed = 0;
    private final CoralIntake m_subsystem;
    private final Joystick rightOpJoystick;
    private final DigitalInput beamBreak;
    private final TeleopElevator elevator;

    public TeleopCoralIntake(CoralIntake m_subsystem, Joystick rightOpJoystick, DigitalInput beamBreak, TeleopElevator elevator) {
        this.m_subsystem = m_subsystem;
        this.rightOpJoystick = rightOpJoystick;
        this.beamBreak = beamBreak;
        this.elevator = elevator;

        addRequirements(m_subsystem);
    }

    @Override
    public void execute() {
        if (rightOpJoystick.getRawButton(5) && elevator.getElevatorPosition() < -120.0) {
            speed = 4;
        } else if (rightOpJoystick.getRawButton(5) && elevator.getElevatorPosition() > -120.0) {
            speed = 3;
        } else if (!beamBreak.get()) {
            speed = 1.2;
        } else if (rightOpJoystick.getRawButton(6)) {
            speed = -1;
        } else {
            speed = 0;
        }
        m_subsystem.setMotorSpeed(speed);
    }


}
