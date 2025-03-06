// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.TeleopClimb;
import frc.robot.commands.TeleopCoralIntake;
import frc.robot.commands.TeleopElevator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final DigitalInput beamBreak = new DigitalInput(Constants.CoralIntakeConstants.beamBreakPort);

    private final Joystick leftDriveJoystick = new Joystick(0);
    private final Joystick rightDriveJoystick = new Joystick(3);
    private final Joystick leftOpJoystick = new Joystick(1);
    private final Joystick rightOpJoystick = new Joystick(2);

    private final JoystickButton leftDriveButton6 = new JoystickButton(leftDriveJoystick, 6);
    private final JoystickButton leftDriveButton4 = new JoystickButton(leftDriveJoystick, 4);
    private final JoystickButton leftDriveButton3 = new JoystickButton(leftDriveJoystick, 3);
    private final JoystickButton rightDriveTrigger = new JoystickButton(rightDriveJoystick, 1);

    private final JoystickButton leftOpButton7 = new JoystickButton(leftOpJoystick, 7);
    private final JoystickButton leftOpButton8 = new JoystickButton(leftOpJoystick, 8);
    private final JoystickButton leftOpButton9 = new JoystickButton(leftOpJoystick, 9);
    private final JoystickButton leftOpButton10 = new JoystickButton(leftOpJoystick, 10);
    private final JoystickButton leftOpButton11 = new JoystickButton(leftOpJoystick, 11);
    private final JoystickButton leftOpButton12 = new JoystickButton(leftOpJoystick, 12);
    private final JoystickButton leftOpTrigger = new JoystickButton(leftOpJoystick, 1);

    private final JoystickButton rightOpButton7 = new JoystickButton(rightOpJoystick, 7);
    private final JoystickButton rightOpButton8 = new JoystickButton(rightOpJoystick, 8);
    private final JoystickButton rightOpButton9 = new JoystickButton(rightOpJoystick, 9);
    private final JoystickButton rightOpButton10 = new JoystickButton(rightOpJoystick, 10);
    private final JoystickButton rightOpButton11 = new JoystickButton(rightOpJoystick, 11);
    private final JoystickButton rightOpButton12 = new JoystickButton(rightOpJoystick, 12);
    private final JoystickButton rightOpTrigger = new JoystickButton(rightOpJoystick, 1);

    private final Climb s_Climb = new Climb(new TalonFX(Constants.ClimberConstants.climberMotorPort));
    private final CoralIntake s_CoralIntake = new CoralIntake(new TalonFX(Constants.CoralIntakeConstants.coralIntakeMotorPort));
    private final Elevator s_Elevator = new Elevator(new TalonFX(Constants.ElevatorConstants.leftElevatorMotorPort), new TalonFX(Constants.ElevatorConstants.rightElevatorMotorPort));
    private final Limelight s_Limelight = new Limelight();
    private final Drive s_Drive = new Drive(drivetrain, leftDriveJoystick, MaxSpeed, MaxAngularRate, drive, rightDriveJoystick, s_Limelight);
    private final Path s_Path = new Path();

    public RobotContainer() {
        configureBindings();

        s_Climb.setDefaultCommand(new TeleopClimb(leftOpJoystick, s_Climb));
        s_CoralIntake.setDefaultCommand(new TeleopCoralIntake(s_CoralIntake, rightOpJoystick, beamBreak));
        s_Elevator.setDefaultCommand(new TeleopElevator(s_Elevator, leftOpJoystick, new DigitalInput(Constants.ElevatorConstants.bottomLimitSwitchPort)));
    }

    private void configureBindings() {
        //blue alliance
//        rightOpButton7.onTrue(s_Path.goToPose(new Pose2d(3.542, 5.577, Rotation2d.fromDegrees(-60.0))));
//        rightOpButton8.onTrue(s_Path.goToPose(new Pose2d(2.588, 4.110, Rotation2d.fromDegrees(0))));
//        rightOpButton9.onTrue(s_Path.goToPose(new Pose2d(3.557, 2.462, Rotation2d.fromDegrees(60.0))));
//        rightOpButton10.onTrue(s_Path.goToPose(new Pose2d(5.452, 2.447, Rotation2d.fromDegrees(120.0))));
//        rightOpButton11.onTrue(s_Path.goToPose(new Pose2d(6.332, 3.963, Rotation2d.fromDegrees(180.0))));
//        rightOpButton12.onTrue(s_Path.goToPose(new Pose2d(5.404, 5.630, Rotation2d.fromDegrees(-120.0))));
//        rightOpTrigger.onTrue(s_Path.goToPose(new Pose2d(1.268, 6.927, Rotation2d.fromDegrees(-55.0))));

        //red alliance
//        rightOpButton7.onTrue(s_Path.goToPose(new Pose2d(14.097, 2.376, Rotation2d.fromDegrees(120.0))));
//        rightOpButton8.onTrue(s_Path.goToPose(new Pose2d(14.971, 4.004, Rotation2d.fromDegrees(180.0))));
//        rightOpButton9.onTrue(s_Path.goToPose(new Pose2d(14.015, 5.632, Rotation2d.fromDegrees(-120.0))));
//        rightOpButton10.onTrue(s_Path.goToPose(new Pose2d(12.129, 5.623, Rotation2d.fromDegrees(-60.0))));
//        rightOpButton11.onTrue(s_Path.goToPose(new Pose2d(11.200, 4.096, Rotation2d.fromDegrees(0))));
//        rightOpButton12.onTrue(s_Path.goToPose(new Pose2d(12.110, 2.422, Rotation2d.fromDegrees(60.0))));
//        rightOpTrigger.onTrue(s_Path.goToPose(new Pose2d(16.275, 1.137, Rotation2d.fromDegrees(125.0))));



        leftDriveButton4.whileTrue(drivetrain.applyRequest(() -> brake));
        leftDriveButton3.whileTrue(drivetrain.applyRequest(() ->
                point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        leftDriveButton6.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));



        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
