// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shooter extends SubsystemBase {
  
  //Motor Controllers
  private final TalonFX shooterMotor = new TalonFX(ShooterConstants.portShooterMotor);

  // Class Variables
  private int inShooterTreshold = 0;
  private double shooterErrorTreshold = 30;
  private final int shooterSettleLoops = 25;

  // Shooter gains
  double newShooterConstant1;
  double previousShooterConstant1 = 0;
  double newShooterConstant2;
  double previousShooterConstant2 = 0;
  double newShooterConstant3;
  double previousShooterConstant3 = 0;
  double newShooterConstant4;
  double previousShooterConstant4 = 0;
  double newShooterConstant5;
  double previousShooterConstant5 = 0;
  double newShooterConstant6;
  double previousShooterConstant6 = 0;
  double newShooterConstant7;
  double previousShooterConstant7 = 0;

  /** Creates a new Shooter. */
  public Shooter() {
    // Set inverted motor
    shooterMotor.setInverted(ShooterConstants.shooterMotorInverted);
    
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

    SmartDashboard.putNumber("ShooterConstant1", newShooterConstant1);
    SmartDashboard.putNumber("ShooterConstant2", newShooterConstant2);
    SmartDashboard.putNumber("ShooterConstant3", newShooterConstant3);
    SmartDashboard.putNumber("ShooterConstant4", newShooterConstant4);
    SmartDashboard.putNumber("ShooterConstant5", newShooterConstant5);
    SmartDashboard.putNumber("ShooterConstant6", newShooterConstant6);
    SmartDashboard.putNumber("ShooterConstant7", newShooterConstant7);
  }
  
  /** 
   * Uses integrated PID for shooter output
   * @param shooterSetpoint double for speed (RPM) setpoint for shooter 
   */
  public void setShooter(double shooterSetpoint){
    shooterMotor.set(ControlMode.Velocity, shooterSetpoint * 2048 / 600);

    SmartDashboard.putNumber("ShooterTarget", shooterSetpoint * 2048 / 600);

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
    // return filter.calculate(shooterMotor.getSelectedSensorVelocity());
    return shooterMotor.getSelectedSensorVelocity();
  }
  
  /**
   *  Get flywheel speed
   * @return Returns ShooterSpeed
   */
  public double getShooterTargetSpeed(){
    if (Robot.tD <= 120) {
      return 0.001365 * Math.pow(Robot.tD, 2) + Robot.tD * 1.175 + 1458 + previousShooterConstant1;
    } else if (Robot.tD <= 288) {
      return 0.001365 * Math.pow(Robot.tD, 2) + Robot.tD * 1.175 + 1458 + previousShooterConstant2;
    } else if (Robot.tD <= 338) {
      return 0.001365 * Math.pow(Robot.tD, 2) + Robot.tD * 1.175 + 1458 + previousShooterConstant3;
    } else if (Robot.tD <= 388){
      return 0.001365 * Math.pow(Robot.tD, 2) + Robot.tD * 1.175 + 1458 + previousShooterConstant4;
    } else if (Robot.tD <= 438) {
      return 0.001365 * Math.pow(Robot.tD, 2) + Robot.tD * 1.175 + 1458 + previousShooterConstant5;
    } else if (Robot.tD <= 488) {
      return 0.001365 * Math.pow(Robot.tD, 2) + Robot.tD * 1.175 + 1458 + previousShooterConstant6;
    } else if (Robot.tD <= 538) {
      return 0.001365 * Math.pow(Robot.tD, 2) + Robot.tD * 1.175 + 1458 + previousShooterConstant7;
    } else {
      return 0.001365 * Math.pow(Robot.tD, 2) + Robot.tD * 1.175 + 1458;
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run  

    Shuffleboard.getTab("Match").add("Shooter Status", shooterIsFinished())
    .withWidget(BuiltInWidgets.kBooleanBox).withSize(1,1).withPosition(3,0);
    Shuffleboard.getTab("Match").add("Shooter Target",shooterMotor.getClosedLoopTarget())
    .withWidget(BuiltInWidgets.kTextView).withSize(1,1).withPosition(3,1);
    Shuffleboard.getTab("Match").add("Shooter Velocity",getShooterSpeed())
    .withWidget(BuiltInWidgets.kTextView).withSize(1,1).withPosition(3,2);
  
    
  }
}
