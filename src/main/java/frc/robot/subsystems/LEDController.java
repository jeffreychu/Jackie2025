// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ScoringSubsystem.ScoringStates;;

public class LEDController extends SubsystemBase {
    private final int numLEDs = 61;
    private final int numLEDsNonCandle = 53;

    private CANdle candle;
    private LEDStates ledState;
    private LEDStates prevLedState;
    private boolean prevRumbleState;
    private boolean currentRumblestate;
    private double currentTime;
    private LEDStates prevState;

    public enum LEDStates {
        DEFAULT,
        OUTAKING,
        INTAKING,
        STORED_RIGHT,
        STORED_LEFT,
        LOADED,
        DISABLED,
        NONE

    }

    public LEDController() {
        candle = new CANdle(Constants.LED.CANdleID, "rio");
        ledState = LEDStates.NONE;
        prevLedState = LEDStates.NONE;
        prevRumbleState = false;
        currentRumblestate = false;
        currentTime = 0;

        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Off;
        candle.configAllSettings(configAll, 100);
    }

    public synchronized void setLEDState(LEDStates state) {
        if (ledState != LEDStates.NONE) {
            ledState = state;
        }
    }

    public synchronized void overrideLEDState(LEDStates state) {
        ledState = state;
    }

    public LEDStates getLEDState() {
        return ledState;
    }

    public void setSolidColor(int r, int g, int b, int w) {
        candle.animate(null);
        candle.setLEDs(r, g, b, w, 0, numLEDs);
    }

    public void setColorBlue() {
        candle.animate(null);
        candle.setLEDs(0, 0, 255, 0, 0, numLEDs);
    }

    public void setColorRed() {
        candle.animate(null);
        candle.setLEDs(255, 0, 0, 0, 0, numLEDs);
    }

    public void setColorGreen() {
        candle.animate(null);
        candle.setLEDs(0, 255, 0, 0, 0, numLEDs);
    }

    public void setColorYellow() {
        candle.animate(null);
        candle.setLEDs(255, 255, 0, 0, 0, numLEDs);
    }

    public void setColorPurple() {
        candle.animate(null);
        candle.setLEDs(255, 0, 255, 0, 0, numLEDs);
    }

    public void setColorOrange() {
        candle.animate(null);
        candle.setLEDs(255, 92, 0, 0, 0, numLEDs);
    }

    public void setColorWhite() {
        candle.animate(null);
        candle.setLEDs(255, 255, 255, 0, 0, numLEDs);
    }

    public void setColorWhiteRight() {
        candle.animate(null);
        candle.setLEDs(255, 255, 255, 0, 26, 18);
        candle.setLEDs(0, 0, 0, 0, 0, 26);
    }

    public void setColorWhiteLeft() {

        candle.animate(null);
        candle.setLEDs(255, 255, 255, 0, 0, 26);
        candle.setLEDs(0, 0, 0, 0, 26, 18);
    }
    
    public void setColorGreenBlink() {
        candle.animate(new StrobeAnimation(0, 255, 0, 0, 0.3, numLEDs));
    }
    
    public void setColorOrangeBlink() {
        candle.animate(new StrobeAnimation(255, 92, 0, 0, 0.3, numLEDs));
    }
    public void setColorYellowBlink() {
        candle.animate(new StrobeAnimation(255, 255, 0, 0, 0.3, numLEDs));
    }
    
    public void setColorBlueBlink() {
        candle.animate(new StrobeAnimation(0, 0, 255, 0, 0.3, numLEDs));
        
    }
    
    public void setColorWhiteBlink() {
        candle.animate(new StrobeAnimation(255, 255, 255, 0, 0.3, numLEDs));
    }
    
    public void setColorCyan(){
        candle.animate(null);
        candle.setLEDs(0, 255, 255, 0, 0, numLEDs);
    }

    // public void setColorCyan(){
    //     candle.animate(null);
    //     candle.setLEDs(0, 255, 255, 0, 0, numLEDs);
    // }
    
    public void setColorCyanBlink() {
        candle.animate(new StrobeAnimation(0, 255, 255, 0, 0.3, numLEDs));
    }

    public void setColorRedBlink() {
        candle.animate(new StrobeAnimation(255, 0, 0, 0, 0.3, numLEDs));
    }

    public void blinkLEDs(int r, int g, int b, int w, double speed) {
        candle.animate(new StrobeAnimation(r, g, b, w, speed, numLEDs));
    }

    public void setLarsonAnmation(int r, int g, int b, int w, double speed, BounceMode mode, int size, int offset) {
        candle.animate(new LarsonAnimation(r, g, b, w, speed, numLEDsNonCandle, mode, size, offset));
    }

    public void setColorFlowAnimation(int r, int g, int b, int w, double speed, Direction direction) {
        candle.animate(new ColorFlowAnimation(r, g, b, w, speed, numLEDs, direction, 0));
    }

    public void setFireAnimation(double brightness, double speed, double sparking, double cooling,
            boolean reverseDirection) {
        candle.animate(new FireAnimation(brightness, speed, numLEDs, sparking, cooling, reverseDirection, 0));
    }

    public void setSingleFadeAnimation(int r, int g, int b, int w, double speed) {
        candle.animate(new SingleFadeAnimation(r, g, b, w, speed, numLEDs, 0));
    }

    public void setRainbowAnimation(double brightness, double speed, boolean reverseDirection) {
        candle.animate(new RainbowAnimation(brightness, speed, numLEDs, reverseDirection, 0));
    }

    public void setDisabledMode() {
        candle.setLEDs(numLEDs, numLEDs, numLEDs, 0, 0, numLEDs);
    }

    private void updateStates() {
        if (Robot.isDisabled) {
            ledState = LEDStates.DISABLED;
        } else if (RobotContainer.operatorJoystick.rightBumper().getAsBoolean()){
            ledState = LEDStates.STORED_RIGHT;
            prevState = LEDStates.STORED_RIGHT;
        } else if (RobotContainer.operatorJoystick.leftBumper().getAsBoolean()){
            ledState = LEDStates.STORED_LEFT;
            prevState = LEDStates.STORED_LEFT;
        } else if (RobotContainer.coralShooter.isLoaded()) {
            ledState = LEDStates.LOADED;
        } else if (RobotContainer.coralShooter.getScoringState() == ScoringStates.OUTAKE) {
            if (prevState != null) {
                ledState = prevState;
            } else {
                ledState = LEDStates.DEFAULT;
            }
        } else {
            // ledState = LEDStates.DEFAULT;
        }
    }

    @Override
    public void periodic() {
        updateStates();
        
        switch (ledState) {
            case DISABLED:
                setLarsonAnmation(255, 255 , 255, 0, 0.5, LarsonAnimation.BounceMode.Center, numLEDsNonCandle, 8);
                break;
            case STORED_RIGHT:
                
                setColorWhiteRight();
                break;
            case STORED_LEFT:
                setColorWhiteLeft();
                break;        
            case INTAKING:
                setColorBlueBlink();
                break;
            case OUTAKING:
                setColorYellowBlink();
                break;
            case LOADED:
                setColorPurple();
                break;
            case DEFAULT: 
            default:
                setColorBlue();
                break;
        }
    }
}