// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.IntakingCommand;
import frc.robot.commands.SetElevatorStateCommand;
import frc.robot.commands.SetScoringStateCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.ScoringSubsystem.ScoringStates;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorStates;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                  // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                      // second
    // max angular velocity // increase RadiansPerSecond to increase turning speed

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                     // motors

    private final SwerveRequest.FieldCentricFacingAngle driveAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverJoystick = new CommandXboxController(Constants.kDriverController);
    private final CommandXboxController operatorJoystick = new CommandXboxController(Constants.kOperatorController);

    public final static ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final static ScoringSubsystem coralShooter = new ScoringSubsystem();
    public final LEDController ledController = new LEDController();
    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //NAMED COMMANDS FOR AUTO
    

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // This is due to X and Y being in reference to field this Field relative
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive
                                                                                                         // forward
                                                                                                         // with
                                                                                                         // negative
                                                                                                         // Y
                                                                                                         // (forward)
                        .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with
                                                                              // negative X
                                                                              // (left)
                        .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive
                                                                                          // counterclockwise
                                                                                          // with
                                                                                          // negative
                                                                                          // X
                                                                                          // (left)
                ));
        // elevator.setDefaultCommand(Commands.run(() ->
        // elevator.setElevatorState(ElevatorStates.HOME)));

        // driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverJoystick.b().whileTrue(drivetrain.applyRequest(
        // () -> point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(),
        // -driverJoystick.getLeftX()))));

        // Commands.run(() -> elevator.setElevatorState(ElevatorStates.INTAKE)
        driverJoystick.rightBumper().whileTrue(
                new ParallelCommandGroup(
                        Commands.runOnce(() -> elevator.setElevatorState(ElevatorStates.INTAKE),
                                elevator),
                        Commands.run(() -> coralShooter.setScoringState(ScoringStates.INTAKE),
                                coralShooter)))
                .onFalse(Commands.run(() -> coralShooter.setScoringState(ScoringStates.NONE),
                        coralShooter));
        // operatorJoystick.leftTrigger(0.1).whileTrue(new ParallelCommandGroup(
        // new RunCommand(() -> coralShooter.setScoringState(ScoringStates.ALGAE),
        // coralShooter)
        // )).onFalse(
        // new ParallelCommandGroup(
        // // new RunCommand(() -> elevator.setElevatorState(ElevatorStates.HOME),
        // elevator),
        // new RunCommand(() -> coralShooter.setScoringState(ScoringStates.NONE),
        // coralShooter)
        // ));

        driverJoystick.leftBumper().whileTrue(
                new RunCommand(() -> coralShooter.setScoringState(ScoringStates.OUTAKE), coralShooter))
                .onFalse(
                        new RunCommand(() -> coralShooter.setScoringState(ScoringStates.NONE),
                                coralShooter));

        driverJoystick.leftTrigger().whileTrue(
                new RunCommand(() -> coralShooter.setScoringState(ScoringStates.OUTAKE2), coralShooter))
                .onFalse(
                        new RunCommand(() -> coralShooter.setScoringState(ScoringStates.NONE),
                                coralShooter));

        operatorJoystick.x().whileTrue(Commands.runOnce(() -> elevator.setElevatorState(ElevatorStates.HOME)));

        operatorJoystick.a().whileTrue(Commands.runOnce(() -> elevator.setElevatorState(ElevatorStates.L1)));
        // .onFalse(Commands.run(() -> elevator.setElevatorState(ElevatorStates.HOME)));

        operatorJoystick.b().whileTrue(Commands.runOnce(() -> elevator.setElevatorState(ElevatorStates.L2)));
        // .onFalse(Commands.run(() -> elevator.setElevatorState(ElevatorStates.HOME)));

        operatorJoystick.y().whileTrue(Commands.runOnce(() -> elevator.setElevatorState(ElevatorStates.L3)));
        // .onFalse(Commands.run(() -> elevator.setElevatorState(ElevatorStates.HOME)));

        // operatorJoystick.x().whileTrue(Commands.run(() ->
        // elevator.setElevatorState(ElevatorStates.L3Algae)))
        // .onFalse(Commands.run(() -> elevator.setElevatorState(ElevatorStates.HOME)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverJoystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
