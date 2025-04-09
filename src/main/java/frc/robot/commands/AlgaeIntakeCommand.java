// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AlgaeSubsystem;
//import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.AlgaeSubsystem.AlgaeStates;
//import frc.robot.subsystems.ScoringSubsystem.ScoringStates;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeIntakeCommand extends Command {
  /** Creates a new IntakingCommand. */
  private AlgaeSubsystem algaeSubsystem;
  private AlgaeStates state;
  //private boolean isLoaded;
  
  public AlgaeIntakeCommand(AlgaeStates state) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algaeSubsystem = RobotContainer.algaeShooter;
    this.state = state;
  //  isLoaded = false;
    addRequirements(algaeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeSubsystem.setAlgaeState(state);
  //  algaeSubsystem.setAlgaeState(AlgaeStates.INTAKE);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //  isLoaded = algaeSubsystem.isLoaded();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // algaeSubsystem.setAlgaeState(AlgaeStates.NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //  return isLoaded;
   return true;

  }
}
