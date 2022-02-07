// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {

  // Motor Controllers
  private final VictorSPX hoodMotor = new VictorSPX(HoodConstants.portHoodMotor);
  
  // Sensors
  // private final Encoder hoodEncoder = new Encoder(ShooterConstants.hoodEncoderPorts[0], ShooterConstants.hoodEncoderPorts[1]);

  // PID Controllers
  private final PIDController hoodPID = new PIDController(HoodConstants.hoodkP, HoodConstants.hoodkI, 
                                                          HoodConstants.hoodkD);

  /** Creates a new Hood. */
  public Hood() {}

  /** 
   * PID Control for hood output
   * @param hoodSetpoint double for hood setpoint 
   */
  public void setHood(double hoodSetpoint){
    hoodMotor.set(ControlMode.PercentOutput, hoodPID.calculate(getHoodAngle(), hoodSetpoint), 
                  DemandType.ArbitraryFeedForward, HoodConstants.hoodkF);
  }

  /** 
   * Manual Hood Output
   * @param hoodSetpoint double for hood setpoint 
   */
  public void setHoodManual(double hoodOutput){
    hoodMotor.set(ControlMode.PercentOutput, hoodOutput);
  }

  /**
   *  Get hood angle
   * @return Returns Hood Position
   */
  public double getHoodAngle(){
    // return hoodEncoder.getPosition();
    return 50;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
