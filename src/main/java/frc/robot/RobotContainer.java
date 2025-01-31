// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.TeleopCoralIntake;
import frc.robot.commands.TeleopElevator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Paths;

public class RobotContainer {
    //private final SendableChooser<Command> autoChooser;

    private final CoralIntake s_CoralIntake = new CoralIntake(new TalonFX(Constants.CoralIntakeConstants.coralIntakeMotorPort));
    private final Elevator s_Elevator = new Elevator(new TalonFX(Constants.ElevatorConstants.leftElevatorMotorPort), new TalonFX(Constants.ElevatorConstants.rightElevatorMotorPort));

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final Joystick leftDriveJoystick = new Joystick(0);
    private final Joystick rightDriveJoystick = new Joystick(3);
    private final Joystick leftOpJoystick = new Joystick(2);
    private final Joystick rightOpJoystick = new Joystick(1);

    private final JoystickButton leftDriveButton3 = new JoystickButton(leftDriveJoystick, 3);
    private final JoystickButton leftDriveButton4 = new JoystickButton(leftDriveJoystick, 4);
    private final JoystickButton leftDriveButton6 = new JoystickButton(leftDriveJoystick, 6);

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

    private final DigitalInput topBreakBeam = new DigitalInput(Constants.CoralIntakeConstants.topBreakBeamPort);
    private final Encoder elevatorEncoder = new Encoder(Constants.ElevatorConstants.elevatorEncoderPortA, Constants.ElevatorConstants.elevatorEncoderPortB);
    private final DigitalInput topLimitSwitch = new DigitalInput(Constants.ElevatorConstants.topLimitSwitchPort);
    private final DigitalInput bottomLimitSwitch = new DigitalInput(Constants.ElevatorConstants.bottomLimitSwitchPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Paths m_Paths = new Paths(drivetrain);

    public RobotContainer() {
        configureBindings();
        s_CoralIntake.setDefaultCommand(new TeleopCoralIntake(s_CoralIntake, rightOpJoystick));
        s_Elevator.setDefaultCommand(new TeleopElevator(s_Elevator, leftOpJoystick, elevatorEncoder, topLimitSwitch, bottomLimitSwitch));
    }

    private void configureBindings() {

        Constants.initialize();

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-leftDriveJoystick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-leftDriveJoystick.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-rightDriveJoystick.getX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );


        leftOpButton7.onTrue(
                m_Paths.goToPose(new Pose2d(6.198, 3.941, Rotation2d.fromDegrees(180)
                )));
        leftOpButton8.onTrue(
                m_Paths.goToPose(new Pose2d(5.391, 2.543, Rotation2d.fromDegrees(120)
                )));
        leftOpButton9.onTrue(
                m_Paths.goToPose(new Pose2d(3.994, 2.946, Rotation2d.fromDegrees(60)
                )));
        leftOpButton10.onTrue(
                m_Paths.goToPose(new Pose2d(2.684, 4.002, Rotation2d.fromDegrees(0)
                )));
        leftOpButton11.onTrue(
                m_Paths.goToPose(new Pose2d(5.617, 5.542, Rotation2d.fromDegrees(-60)
                )));
        leftOpButton12.onTrue(
                m_Paths.goToPose(new Pose2d(5.368, 5.507, Rotation2d.fromDegrees(-120)
                )));
        leftOpTrigger.onTrue(
                m_Paths.goToPose(new Pose2d(1.274, 1.108, Rotation2d.fromDegrees(-125)
                )));

        rightOpButton7.onTrue(
                m_Paths.goToPose(new Pose2d(14.808, 4.002, Rotation2d.fromDegrees(180)
                )));
        rightOpButton8.onTrue(
                m_Paths.goToPose(new Pose2d(13.909, 2.520, Rotation2d.fromDegrees(120)
                )));
        rightOpButton9.onTrue(
                m_Paths.goToPose(new Pose2d(12.182, 2.473, Rotation2d.fromDegrees(60)
                )));
        rightOpButton10.onTrue(
                m_Paths.goToPose(new Pose2d(11.272, 4.037, Rotation2d.fromDegrees(0)
                )));
        rightOpButton11.onTrue(
                m_Paths.goToPose(new Pose2d(12.182, 5.589, Rotation2d.fromDegrees(-60)
                )));
        rightOpButton12.onTrue(
                m_Paths.goToPose(new Pose2d(13.991, 5.530, Rotation2d.fromDegrees(-120)
                )));
        rightOpTrigger.onTrue(
                m_Paths.goToPose(new Pose2d(16.313, 1.154, Rotation2d.fromDegrees(-60)
                )));

        leftDriveButton3.whileTrue(drivetrain.applyRequest(() -> brake));
        leftDriveButton4.whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-leftDriveJoystick.getY(), -leftDriveJoystick.getX()))
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
        return new PathPlannerAuto("aroundCoralAuto");
    }
}
