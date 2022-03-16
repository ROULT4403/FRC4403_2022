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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.HoodConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  double newHoodAngle1;
  double previousHoodAngle1 = 0;
  double newHoodAngle2;
  double previousHoodAngle2 = 0;
  double newHoodAngle3;
  double previousHoodAngle3 = 5;
  double newHoodAngle4;
  double previousHoodAngle4 = 13;
  double newHoodAngle5;
  double previousHoodAngle5 = 25;
  double newHoodAngle6;
  double previousHoodAngle6 = 33;
  double newHoodAngle7;
  double previousHoodAngle7 = 41;

  /** Creates a new Hood. */
  public Hood() {
    // Set inverted motor
    hoodMotor.setInverted(HoodConstants.hoodMotorInverted);
    hoodMotor.setNeutralMode(NeutralMode.Brake);

    SmartDashboard.putNumber("HoodAngle1", newHoodAngle1);
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
  public double getHoodTargetAngle(){
    if (Robot.tD <= 120) {
      return previousHoodAngle1;
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
   
    Shuffleboard.getTab("Match").add("HoodAngle",getHoodAngle())
    .withWidget(BuiltInWidgets.kDial).withSize(1,1).withPosition(5,1);
    Shuffleboard.getTab("Match").add("HoodTargetAngle",getHoodTargetAngle())
    .withWidget(BuiltInWidgets.kDial).withSize(1,1).withPosition(5,2);
    
  }
}
