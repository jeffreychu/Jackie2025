// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new Limelight. */
  
  private final String LLname;


  public LimelightSubsystem() {
    LLname = Constants.Limelight.kLimelightName;
  }

  public Pose2d getRecentTagPose2d(){
    return LimelightHelpers.getLatestResults(LLname).getBotPose2d();
  }
  
  public Pose3d getRecentTagPose3d(){
    return LimelightHelpers.getLatestResults(LLname).getBotPose3d();
  }

  public Rotation2d getRecentTagRotation2d(){
    return LimelightHelpers.getLatestResults(LLname).getBotPose2d().getRotation();
  }

  public Rotation3d getRecentTagRotation3d(){
    return LimelightHelpers.getLatestResults(LLname).getBotPose3d().getRotation();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LimelightHelpers.Flush();
  }
  
}
