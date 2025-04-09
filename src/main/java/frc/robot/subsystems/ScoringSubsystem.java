// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ScoringSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */

  public enum ScoringStates {
    NONE,
    INTAKE,
    OUTAKE,
    OUTAKE2,
    OUTAKE2AUTO,
    ALGAE,
  }

  private ScoringStates currentState;
  private TalonFX scoringMotor;
  private CANrange leftSensor, rightSensor;
  private double sensorThreshold;
  private Debouncer debouncer;

  private double prevTime = Timer.getFPGATimestamp();
  public ScoringSubsystem() {
    scoringMotor = new TalonFX(Constants.Scoring.Motors.scoringMotorID, "canivore");

    leftSensor = new CANrange(Constants.Scoring.Sensors.leftSensorID, "canivore");
    rightSensor = new CANrange(Constants.Scoring.Sensors.rightSensorID, "canivore");

    currentState = ScoringStates.NONE;

    sensorThreshold = .10;
    debouncer = new Debouncer(.025);
  }

  public void setScoringSpeed(double speed, boolean isLoaded) {
    if (isLoaded == false) {
      scoringMotor.set(speed);
    } else {
      scoringMotor.set(0.0);
    }
  }

  public void setScoringState(ScoringStates state) {
    this.currentState = state;
  }

  public ScoringStates getScoringState() {
    return currentState;
  }

  public boolean isLoaded() {
    return (leftSensor.getDistance().getValueAsDouble() < sensorThreshold
        || rightSensor.getDistance().getValueAsDouble() < sensorThreshold);
  }

  @Override
  public void periodic() {
    if (isLoaded()) {
      if (Timer.getFPGATimestamp() - prevTime < 1.0) {
        System.out.println("If executed");
        RobotContainer.driverJoystick.setRumble(RumbleType.kBothRumble, 0);
        RobotContainer.operatorJoystick.setRumble(RumbleType.kBothRumble, Constants.driverRumbleValue);
      } else {
        System.out.println("Else executed");
        RobotContainer.driverJoystick.setRumble(RumbleType.kBothRumble, 0);
        RobotContainer.operatorJoystick.setRumble(RumbleType.kBothRumble, 0);
      }
    } else {
      prevTime = Timer.getFPGATimestamp();
    }
     

    // This method will be called once per scheduler run

    switch (currentState) {
      case NONE:
        setScoringSpeed(0.0, false);
        break;
      case INTAKE:
        setScoringSpeed(Constants.Scoring.ScoringSpeeds.coralIntake, isLoaded());
        break;
      case OUTAKE:
        setScoringSpeed(Constants.Scoring.ScoringSpeeds.coralOutake, false);
        break;
      case OUTAKE2:
        setScoringSpeed(Constants.Scoring.ScoringSpeeds.coralOutake2, false);
        break;
      case OUTAKE2AUTO:
        setScoringSpeed(Constants.Scoring.ScoringSpeeds.coralOutake2Auto, false);
        break;
      case ALGAE:
        setScoringSpeed(Constants.Scoring.ScoringSpeeds.coralAlgae, false);
        break;
      default:
        break;
    }
  }

}
