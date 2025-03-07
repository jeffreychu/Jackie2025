
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  //testing git

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic("poseFromLL", Pose2d.struct).publish();

  private Command m_autonomousCommand;
  public static boolean isDisabled = false;

  private final RobotContainer m_robotContainer;


  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (Constants.Limelight.kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      // JEFFREY: this sets the robot orientation of the robot during auton and updates odometry
      LimelightHelpers.SetRobotOrientation(Constants.Limelight.kLimelightName, headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight.kLimelightName);
      if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        publisher.set(llMeasurement.pose);
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
      }
    }
  }

  @Override
  public void disabledInit() {
    isDisabled = true;
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.elevator.zeroSensors();
    RobotContainer.climb.zeroSensors();
  }

  @Override
  public void disabledExit() {
    isDisabled = false;
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
