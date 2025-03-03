// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.ScoringSubsystem.ScoringStates;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetScoringStateCommand extends Command {
  /** Creates a new SetScoringStateCommand. */
  private ScoringSubsystem scoringSubsystem;
  private ScoringStates state;

  public SetScoringStateCommand(ScoringStates state) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.scoringSubsystem = RobotContainer.coralShooter;
    this.state = state;
    addRequirements(scoringSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    scoringSubsystem.setScoringState(state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
