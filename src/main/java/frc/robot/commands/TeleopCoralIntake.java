package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class CoralIntake extends Command {
    private final DigitalInput topBreakBeam;
    public CoralIntake(DigitalInput topBreakBeam) {
        this.topBreakBeam = topBreakBeam;
    }
    
    @Override
    public void execute() {
        SmartDashboard.putBoolean("Top Break Beam", topBreakBeam.get());
    }

}
