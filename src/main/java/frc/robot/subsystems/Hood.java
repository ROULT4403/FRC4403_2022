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

  private final DigitalInput hoodLimitSwitch = new DigitalInput(9);

  // PID Controllers
  private final PIDController hoodPID = new PIDController(HoodConstants.hoodkP, HoodConstants.hoodkI, 
                                                          HoodConstants.hoodkD);

  /** Creates a new Hood. */
  public Hood() {
    // Set inverted motor
    hoodMotor.setInverted(HoodConstants.hoodMotorInverted);
    hoodMotor.setNeutralMode(NeutralMode.Brake);
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
    return Robot.tD * 0.165;
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
    SmartDashboard.putBoolean("HoodSwitch", hoodLimitSwitch.get());
    SmartDashboard.putNumber("HoodSetpoint", getHoodTargetAngle());
  }
}
