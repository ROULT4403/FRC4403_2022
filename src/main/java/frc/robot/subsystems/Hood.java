// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {

  // Motor Controllers
  private final VictorSPX hoodMotor = new VictorSPX(HoodConstants.portHoodMotor);
  
  // Sensors
  private final Encoder hoodEncoder = new Encoder(HoodConstants.hoodEncoderPorts[0], HoodConstants.hoodEncoderPorts[1], true, EncodingType.k4X);

  private final DigitalInput hoodLimitSwitch = new DigitalInput(HoodConstants.limitSwitchPort);

  // PID Controllers
  private final PIDController hoodPID = new PIDController(HoodConstants.hoodkP, HoodConstants.hoodkI, 
                                                          HoodConstants.hoodkD);

  // Hood Angles
  double newHoodAngle1 = 0;
  int previousHoodAngle1 = 0;
  double newHoodAngle1_25 = 0;
  int previousHoodAngle1_25 = 0;
  double newHoodAngle1_5 = 0;
  int previousHoodAngle1_5 = 0;
  double newHoodAngle2 = 0;
  int previousHoodAngle2 = 0;
  double newHoodAngle3 = 5;
  int previousHoodAngle3 = 5;
  double newHoodAngle4 = 13;
  int previousHoodAngle4 = 13;
  double newHoodAngle5 = 25;
  int previousHoodAngle5 = 25;
  double newHoodAngle6 = 33;
  int previousHoodAngle6 = 33;
  double newHoodAngle7 = 41;
  int previousHoodAngle7 = 41;

  /** Creates a new Hood. */
  public Hood() {
    // Set inverted motor
    hoodMotor.setInverted(HoodConstants.hoodMotorInverted);
    hoodMotor.setNeutralMode(NeutralMode.Brake);

    SmartDashboard.putNumber("HoodAngle1", newHoodAngle1);
    SmartDashboard.putNumber("HoodAngle1_25", newHoodAngle1_25);
    SmartDashboard.putNumber("HoodAngle1_5", newHoodAngle1_5);
    SmartDashboard.putNumber("HoodAngle2", newHoodAngle2);
    SmartDashboard.putNumber("HoodAngle3", newHoodAngle3);
    SmartDashboard.putNumber("HoodAngle4", newHoodAngle4);
    SmartDashboard.putNumber("HoodAngle5", newHoodAngle5);
    SmartDashboard.putNumber("HoodAngle6", newHoodAngle6);
    SmartDashboard.putNumber("HoodAngle7", newHoodAngle7);
  }
  /** 
   * PID Control for hood output
   * @param hoodSetpoint double for hood setpoint 
   */
  public void setHood(double hoodSetpoint){
    if (getHoodAngle() > 100 && hoodSetpoint > 0){
      hoodMotor.set(ControlMode.PercentOutput, 0);
    } else if (hoodLimitSwitch.get() && hoodSetpoint < 0){
      hoodMotor.set(ControlMode.PercentOutput, 0);
    } else {
      hoodMotor.set(ControlMode.PercentOutput, hoodPID.calculate(getHoodAngle(), hoodSetpoint), 
      DemandType.ArbitraryFeedForward, HoodConstants.hoodkF);
    }
    resetAngle();
  }
  
  /** 
   * Manual Hood Output
   * @param hoodSetpoint double for hood setpoint 
   */
  public void setHoodManual(double hoodOutput) {
    if (getHoodAngle() > 100 && hoodOutput > 0){
      hoodMotor.set(ControlMode.PercentOutput, 0);
    } else if (hoodLimitSwitch.get() && hoodOutput < 0){
      hoodMotor.set(ControlMode.PercentOutput, 0);
    } else {
      hoodMotor.set(ControlMode.PercentOutput, hoodOutput);
    }
    resetAngle();
  }

  /**
   *  Get hood angle
   * @return Returns Hood Position
   */
  public double getHoodAngle(){
    return hoodEncoder.get() * 100 / 2168;
  }
  
  /**
   *  Get hood angle
   * @return Returns Hood Position
   */
  public int getHoodTargetAngle(){
    if (Robot.tD <= 130) {
      return previousHoodAngle1;
    } else if (Robot.tD <= 170) {
      return previousHoodAngle1_25;
    } else if (Robot.tD <= 220) {
      return previousHoodAngle1_5;
    } else if (Robot.tD <= 288) {
      return previousHoodAngle2;
    } else if (Robot.tD <= 338) {
      return previousHoodAngle3;
    } else if (Robot.tD <= 388){
      return previousHoodAngle4;
    } else if (Robot.tD <= 438) {
      return previousHoodAngle5;
    } else if (Robot.tD <= 488) {
      return previousHoodAngle6;
    } else if (Robot.tD <= 538) {
      return previousHoodAngle7;
    } else {
      return 0;
    }

    // return 0.0002499 * Math.pow(Robot.tD, 2) + Robot.tD * -0.0439 + 2.02;
  }

  /**
   * Move hood until Limit Switch is triggered, then reset angle
   */
  public void resetAngle(){
    if (hoodLimitSwitch.get()) {
      hoodEncoder.reset();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("HoodAngle", getHoodAngle());
    SmartDashboard.putNumber("HoodTargetAngle", getHoodTargetAngle());

    // Tune hood values in Shuffleboard
    newHoodAngle1 = SmartDashboard.getNumber("HoodAngle1", 0.0);
    if (newHoodAngle1 != previousHoodAngle1){
      previousHoodAngle1 = (int)newHoodAngle1;
    }

    newHoodAngle1_25 = SmartDashboard.getNumber("HoodAngle1_25", 0.0);
    if (newHoodAngle1_25 != previousHoodAngle1_25){
      previousHoodAngle1_25 = (int)newHoodAngle1_25;
    }

    newHoodAngle1_5 = SmartDashboard.getNumber("HoodAngle1_5", 0.0);
    if (newHoodAngle1_5 != previousHoodAngle1_5){
      previousHoodAngle1_5 = (int)newHoodAngle1_5;
    }
    
    newHoodAngle2 = SmartDashboard.getNumber("HoodAngle2", 0.0);
    if (newHoodAngle2 != previousHoodAngle2){
      previousHoodAngle2 = (int)newHoodAngle2;
    }

    newHoodAngle3 = SmartDashboard.getNumber("HoodAngle3", 0.0);
    if (newHoodAngle3 != previousHoodAngle3){
      previousHoodAngle3 = (int)newHoodAngle3;
    }

    newHoodAngle4 = SmartDashboard.getNumber("HoodAngle4", 0.0);
    if (newHoodAngle4 != previousHoodAngle4){
      previousHoodAngle4 = (int)newHoodAngle4;
    }

    newHoodAngle5 = SmartDashboard.getNumber("HoodAngle5", 0.0);
    if (newHoodAngle5 != previousHoodAngle5){
      previousHoodAngle5 = (int)newHoodAngle5;
    }

    newHoodAngle6 = SmartDashboard.getNumber("HoodAngle6", 0.0);
    if (newHoodAngle6 != previousHoodAngle6){
      previousHoodAngle6 = (int)newHoodAngle6;
    }

    newHoodAngle7 = SmartDashboard.getNumber("HoodAngle7", 0.0);
    if (newHoodAngle7 != previousHoodAngle7){
      previousHoodAngle7 = (int)newHoodAngle7;
    }
  }
}
