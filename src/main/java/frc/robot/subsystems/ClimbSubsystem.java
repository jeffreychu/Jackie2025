// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorStates;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  public enum ClimberStates {
    HOME,
    TARGET
  }

  double setPoint = 0.0; //Set a setpoint
  private TalonFX climberMotor;
  private ClimberStates currentState;
  private DigitalInput climberSensor;
  private double currentMotorPosition;
  private boolean hasZeroed;

  /* Start at position 0, use slot 0 */
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  public ClimbSubsystem() {

    climberMotor = new TalonFX(Constants.Climber.Motors.climberMotorID);
    hasZeroed = false;
    climberSensor = new DigitalInput(Constants.Climber.Sensors.climberSensor);
    currentState = ClimberStates.HOME;


  //change values
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 50.0;
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 12.5;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    //change values
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    climberMotor.getConfigurator().apply(configs);

    climberMotor.setPosition(0);
  }

  public void setClimberState(ClimberStates state) {
    System.out.println("Setting state: " + state);
    this.currentState = state;
  }

  public void setClimberPosition(double climberPos) {
    if(!hasZeroed) {
      return; 
    }

    climberMotor.setControl(m_positionVoltage.withPosition(climberPos));

  }

  public boolean getClimberSensor() {
    return !climberSensor.get();
  }

  public void zeroSensors() {
    if (getClimberSensor()) {
      climberMotor.setPosition(0);
      hasZeroed = true;
    }

    //SmartDashboard.putBoolean("Elevator Sensor", getElevatorSensor());
  }

  public boolean isAtPosition(){
    return climberMotor.getClosedLoopError().getValueAsDouble() <= Constants.Climber.positionThreshold;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (currentState) {
      case HOME:
        setClimberPosition(Constants.Climber.ClimberStatePositions.motorHomePosition);
        break;
      case TARGET:
        setClimberPosition(Constants.Climber.ClimberStatePositions.motorTargetPosition);
        break;
      default:
        break;
    }
  }
}
