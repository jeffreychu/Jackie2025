// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.AlgorithmConstraints;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */

  public enum AlgaeStates {
    NONE,
    INTAKE,
    OUTAKE,
  }

  private AlgaeStates currentState;
  private TalonFX algaeMotor;
  //private CANrange leftSensor, rightSensor;
  //private double sensorThreshold;
  private Debouncer debouncer;

  private double prevTime = Timer.getFPGATimestamp();
  public AlgaeSubsystem() {
    algaeMotor = new TalonFX(Constants.Algae.algaeMotorID, "canivore");
    
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    algaeMotor.getConfigurator().apply(configs);

   // leftSensor = new CANrange(Constants.Scoring.Sensors.leftSensorID, "canivore");
    //rightSensor = new CANrange(Constants.Scoring.Sensors.rightSensorID, "canivore");

    currentState = AlgaeStates.NONE;

    //sensorThreshold = .10;
    debouncer = new Debouncer(.025);
  }

  public void setAlgaeSpeed(double speed, boolean isLoaded) {
    if (isLoaded == false) {
      algaeMotor.set(speed);
    } else {
      algaeMotor.set(0.0);
    }
  }

  public void setAlgaeState(AlgaeStates state) {
    this.currentState = state;
  }

  public AlgaeStates getAlgaeState() {
    return currentState;
  }

  public boolean isLoaded() {
    return true;
 //   return (leftSensor.getDistance().getValueAsDouble() < sensorThreshold
   //     || rightSensor.getDistance().getValueAsDouble() < sensorThreshold);
  }

  @Override
  public void periodic() {
    if (isLoaded()) {
      if (Timer.getFPGATimestamp() - prevTime < 1.0) {
        RobotContainer.driverJoystick.setRumble(RumbleType.kBothRumble, Constants.driverRumbleValue);
        RobotContainer.operatorJoystick.setRumble(RumbleType.kBothRumble, Constants.driverRumbleValue);
      } else {
        RobotContainer.driverJoystick.setRumble(RumbleType.kBothRumble, 0);
        RobotContainer.operatorJoystick.setRumble(RumbleType.kBothRumble, 0);
      }
    } else {
      prevTime = Timer.getFPGATimestamp();
    }

    // This method will be called once per scheduler run

    switch (currentState) {
      case NONE:
        setAlgaeSpeed(0.0, false);
        break;
      case INTAKE:
        setAlgaeSpeed(Constants.Algae.AlgaeSpeeds.algaeIntake1, false);
        break;
      case OUTAKE:
        setAlgaeSpeed(Constants.Algae.AlgaeSpeeds.algaeOutake, false);
        break;
  /*    case OUTAKE2AUTO:
        setScoringSpeed(Constants.Scoring.ScoringSpeeds.coralOutake2Auto, false);
        break;
      case ALGAE:
        setScoringSpeed(Constants.Scoring.ScoringSpeeds.coralAlgae, false);
        break; */
      default:
        break;
    }
  }

}
