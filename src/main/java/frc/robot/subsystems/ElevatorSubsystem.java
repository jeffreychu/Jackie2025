// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  public enum ElevatorStates {
    HOME,
    IDLE,
    INTAKE,
    L1,
    L2,
    L2Algae,
    L3Algae,
    L3
  }

  double setPoint = 10.0;
  private TalonFX elevatorLeft;
  private TalonFX elevatorRight;
  private ElevatorStates currentState;
  private DigitalInput elevatorSensor;
  private double currentLeftPosition;
  private double currentRightPosition;
  private boolean hasZeroed;

  /* Start at position 0, use slot 0 */
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  /** Creates a new Elevator. */
  public ElevatorSubsystem() {
    elevatorLeft = new TalonFX(Constants.Elevator.Motors.elevatorLeftID);
    elevatorRight = new TalonFX(Constants.Elevator.Motors.elevatorRightID); 

    hasZeroed = false;
    elevatorSensor = new DigitalInput(Constants.Elevator.Sensors.elevatorSensor);
    currentState = ElevatorStates.HOME;

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 50.0;
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 12.5;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.Voltage.PeakForwardVoltage = 16;
    configs.Voltage.PeakReverseVoltage = -2;
    configs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1.0;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 18.51;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    elevatorRight.getConfigurator().apply(configs);

    elevatorLeft.setPosition(0);
    elevatorRight.setPosition(0);

    elevatorLeft.setControl(new Follower(elevatorRight.getDeviceID(), false));
  }

  public void setTargetPoint(double targetPoint) {
    setPoint = targetPoint;
  }

  public void setElevatorState(ElevatorStates state) {
    System.out.println("Setting state: " + state);
    this.currentState = state;
  }

  public void setElevatorPosition(double elevatorPos) {
    elevatorRight.setControl(m_positionVoltage.withPosition(elevatorPos));
  }

  public boolean getElevatorSensor() {
    return !elevatorSensor.get();
  }

  public void zeroSensors() {
    if (getElevatorSensor()) {
      elevatorLeft.setPosition(0);
      elevatorRight.setPosition(0);
    }

    SmartDashboard.putBoolean("Elevator Sensor", getElevatorSensor());
  }

  public boolean isAtPosition(){
    return elevatorRight.getClosedLoopError().getValueAsDouble() <= Constants.Elevator.positionThreshold;
  }

  @Override
  public void periodic() {
    zeroSensors();

    currentLeftPosition = elevatorLeft.getPosition().getValueAsDouble();
    currentRightPosition = elevatorRight.getPosition().getValueAsDouble();

  
    switch (currentState) {
      case HOME:
        setElevatorPosition(Constants.Elevator.ElevatorStatePositions.leftHomePostion);
        setElevatorPosition(Constants.Elevator.ElevatorStatePositions.rightHomePostion);
        break;
      case INTAKE:
        setElevatorPosition(Constants.Elevator.ElevatorStatePositions.coralIntakePostion);
        break;
      case L1:
        setElevatorPosition(Constants.Elevator.ElevatorStatePositions.L1Position);
        break;
      case L2:
        setElevatorPosition(Constants.Elevator.ElevatorStatePositions.L2Position);
        break;
      case L3:
        setElevatorPosition(Constants.Elevator.ElevatorStatePositions.L3Position);
        break;
      case L3Algae:
        setElevatorPosition(Constants.Elevator.ElevatorStatePositions.L3AlgaePosition);
        break;
      default:
        break;
    }
  }
}
