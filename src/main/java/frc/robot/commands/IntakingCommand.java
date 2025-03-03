// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.ScoringSubsystem.ScoringStates;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakingCommand extends Command {
  /** Creates a new IntakingCommand. */
  private ScoringSubsystem scoringSubsystem;
  private boolean isLoaded;
  
  public IntakingCommand(ScoringSubsystem scoringSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.scoringSubsystem = scoringSubsystem;
    isLoaded = false;
    addRequirements(scoringSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    scoringSubsystem.setScoringState(ScoringStates.INTAKE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isLoaded = scoringSubsystem.isLoaded();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    scoringSubsystem.setScoringState(ScoringStates.NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isLoaded;
  }
}
