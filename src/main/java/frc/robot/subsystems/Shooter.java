// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.Relay;


public class Shooter extends SubsystemBase {
  
  //Motor Controllers
  private final TalonFX shooterMotor = new TalonFX(ShooterConstants.portShooterMotor);

  // Class Variables
  private int inShooterTreshold = 0;
  private double shooterErrorTreshold = 30;
  private final int shooterSettleLoops = 25;
  
  //Relay LEDS
  private final Relay LED = new Relay(ShooterConstants.relayPort);
  private boolean isToggled = ShooterConstants.relayDefault;
  
  
  /** Creates a new Shooter. */
  public Shooter() {
    // Set inverted motor
    shooterMotor.setInverted(ShooterConstants.shooterMotorInverted);
    
    LED.set(Relay.Value.kForward);
    
    //Configure Talon FX 
    shooterMotor.configFactoryDefault();
    shooterMotor.configPeakOutputReverse(0);
    shooterMotor.configNominalOutputForward(0.1);
    shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shooterMotor.configAllowableClosedloopError(0, 10, 30);
    shooterMotor.config_kP(0, ShooterConstants.shooterkP);
    shooterMotor.config_kI(0, ShooterConstants.shooterkI);
    shooterMotor.config_kD(0, ShooterConstants.shooterkD);
    shooterMotor.config_kF(0, ShooterConstants.shooterkF);
  }
  
  /** 
   * Uses integrated PID for shooter output
   * @param shooterSetpoint double for speed (RPM) setpoint for shooter 
   */
  public void setShooter(double shooterSetpoint){
    shooterMotor.set(ControlMode.Velocity, shooterSetpoint * 2048 / 600);
  }
  
  /** 
   * Uses integrated PID for shooter output
   * @param shooterSetpoint double for speed (RPM) setpoint for shooter 
   */
  public void setShooterManual(double shooterOutput){
    shooterMotor.set(ControlMode.PercentOutput, shooterOutput);
  }
  
  /**
   *  Returns whether controller is at reference
   * @return Returns boolean true if closed loop control is at target 
   */
  public boolean shooterIsFinished() {
    if(shooterMotor.getClosedLoopError() < shooterErrorTreshold && shooterMotor.getClosedLoopError() > -shooterErrorTreshold) {
      ++inShooterTreshold;
    } else {
      inShooterTreshold = 0;
      return false;
    }
    
    return inShooterTreshold > shooterSettleLoops;
    // return false;
  }
  
  /**
   *  Get flywheel speed
   * @return Returns ShooterSpeed
   */
  public double getShooterSpeed(){
    return shooterMotor.getSelectedSensorVelocity();
  }
  
  /**
   *  Get flywheel speed
   * @return Returns ShooterSpeed
   */
  public double getShooterTargetSpeed(){
    return 0.001365 * Math.pow(Robot.tD, 2) + Robot.tD * 1.175 + 1458;
  }
  
  public void LEDToggle() {
    // LED.set(Relay.Value.kForward);
    if (!isToggled) {
      LED.set(Relay.Value.kForward);
    } else {
      LED.set(Relay.Value.kReverse);
    }
    isToggled = !isToggled;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
    SmartDashboard.putNumber("ShooterVelocity", getShooterSpeed());
    SmartDashboard.putNumber("ShooterTarget", shooterMotor.getClosedLoopTarget());
  }
}
