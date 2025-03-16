// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new Limelight. */
  private int lastSeenTagID = -1;
  
  private final String LLname;
  private AprilTagFieldLayout field;


  public LimelightSubsystem() {
    field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
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

  public double getTX() {
    return LimelightHelpers.getTX(LLname);
  }
  public double getTY() {
    return LimelightHelpers.getTY(LLname);
  }

  public Rotation2d getLatestTagRotation() {
    if (lastSeenTagID == -1) {
      return new Rotation2d();
    }
    System.out.println(lastSeenTagID);
    return field.getTagPose(lastSeenTagID).get().getRotation().toRotation2d();
    
  }
  //TODO implement logic for aligning to score and movement there



  @Override
  public void periodic() {
    if (lastSeenTagID != LimelightHelpers.getFiducialID(LLname) && LimelightHelpers.getFiducialID(LLname) != -1 && RobotContainer.driverJoystick.getRightTriggerAxis() < 0.1) {
      lastSeenTagID = (int) LimelightHelpers.getFiducialID(LLname);
    }  
  }
}
 