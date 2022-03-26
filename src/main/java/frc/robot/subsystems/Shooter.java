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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shooter extends SubsystemBase {
  
  //Motor Controllers
  private final TalonFX shooterMotor = new TalonFX(ShooterConstants.portShooterMotor);

  // Class Variables
  private int inShooterTreshold = 0;
  private double shooterErrorTreshold = 35;
  private final int shooterSettleLoops = 50;
  private boolean isFinished = false;

  // Shooter gains
  // double newShooterConstant1;
  double previousShooterConstant1 = 0; //done
  // double newShooterConstant1_25;
  double previousShooterConstant1_25 = -45; //done
  // double newShooterConstant1_5;
  double previousShooterConstant1_5 = -100; //done
  // double newShooterConstant2;
  double previousShooterConstant2 = -100; //done
  // double newShooterConstant3;
  double previousShooterConstant3 = -150; //doneish
  // double newShooterConstant4;
  double previousShooterConstant4 = -67; // In progress
  // double newShooterConstant5;
  double previousShooterConstant5 = -130; // Not final
  // double newShooterConstant6;
  double previousShooterConstant6 = -100; // Not final
  // double newShooterConstant7;
  double previousShooterConstant7 = -100; // Not final

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

    // SmartDashboard.putNumber("ShooterConstant1", newShooterConstant1);
    // SmartDashboard.putNumber("ShooterConstant1_25", newShooterConstant1_25);
    // SmartDashboard.putNumber("ShooterConstant1_5", newShooterConstant1_5);
    // SmartDashboard.putNumber("ShooterConstant2", newShooterConstant2);
    // SmartDashboard.putNumber("ShooterConstant3", newShooterConstant3);
    // SmartDashboard.putNumber("ShooterConstant4", newShooterConstant4);
    // SmartDashboard.putNumber("ShooterConstant5", newShooterConstant5);
    // SmartDashboard.putNumber("ShooterConstant6", newShooterConstant6);
    // SmartDashboard.putNumber("ShooterConstant7", newShooterConstant7);
  }
  
  /** 
   * Uses integrated PID for shooter output
   * @param shooterSetpoint double for speed (RPM) setpoint for shooter 
   */
  public void setShooter(double shooterSetpoint){
    shooterMotor.set(ControlMode.Velocity, shooterSetpoint * 2048 / 600);
    shooterIsFinished();
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
  // public boolean shooterIsFinished() {
  //   if(shooterMotor.getClosedLoopError() < shooterErrorTreshold && shooterMotor.getClosedLoopError() > -shooterErrorTreshold) {
  //     ++inShooterTreshold;
  //   } else {
  //     inShooterTreshold = 0;
  //     return false;
  //   }
    
  //   return inShooterTreshold > shooterSettleLoops;
  // }
  public void shooterIsFinished() {
    if(Math.abs(shooterMotor.getClosedLoopError()) < shooterErrorTreshold) {
      ++inShooterTreshold;
    } else {
      inShooterTreshold = 0;
      isFinished = false;
    }
    
    isFinished = inShooterTreshold > shooterSettleLoops;
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
    if (Robot.tD <= 130) {
      return 0.001365 * Math.pow(Robot.tD, 2) + Robot.tD * 1.175 + 1458 + previousShooterConstant1;
    } else if (Robot.tD <= 170) {
      return 0.001365 * Math.pow(Robot.tD, 2) + Robot.tD * 1.175 + 1458 + previousShooterConstant1_25;
    } else if (Robot.tD <= 220) {
      return 0.001365 * Math.pow(Robot.tD, 2) + Robot.tD * 1.175 + 1458 + previousShooterConstant1_5;
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

  public void setShooterIsFinished() {
    isFinished = false;
    inShooterTreshold = 0;
  }

  public boolean getShooterIsFinished() {
    return isFinished;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
    SmartDashboard.putNumber("ShooterVelocity", getShooterSpeed());
    SmartDashboard.putNumber("ClosedLoopError", shooterMotor.getClosedLoopError());
    // SmartDashboard.putNumber("ShooterTarget", getShooterTargetSpeed());
    SmartDashboard.putBoolean("ShooterIsFinishedInShooter", isFinished);

    SmartDashboard.putNumber("Distance", Robot.tD);
    SmartDashboard.putNumber("VisionYaw", Robot.tX);

    // Tune hood values in Shuffleboard
    // newShooterConstant1 = SmartDashboard.getNumber("ShooterConstant1", 0.0);
    // if (newShooterConstant1 != previousShooterConstant1){
    //   previousShooterConstant1 = newShooterConstant1;
    // }

    // newShooterConstant1_25 = SmartDashboard.getNumber("ShooterConstant1_25", 0.0);
    // if (newShooterConstant1_25 != previousShooterConstant1_25){
    //   previousShooterConstant1_25 = newShooterConstant1_25;
    // }

    // newShooterConstant1_5 = SmartDashboard.getNumber("ShooterConstant1_5", 0.0);
    // if (newShooterConstant1_5 != previousShooterConstant1_5){
    //   previousShooterConstant1_5 = newShooterConstant1_5;
    // }

    // newShooterConstant2 = SmartDashboard.getNumber("ShooterConstant2", 0.0);
    // if (newShooterConstant2 != previousShooterConstant2){
    //   previousShooterConstant2 = newShooterConstant2;
    // }

    // newShooterConstant3 = SmartDashboard.getNumber("ShooterConstant3", 0.0);
    // if (newShooterConstant3 != previousShooterConstant3){
    //   previousShooterConstant3 = newShooterConstant3;
    // }

    // newShooterConstant4 = SmartDashboard.getNumber("ShooterConstant4", 0.0);
    // if (newShooterConstant4 != previousShooterConstant4){
    //   previousShooterConstant4 = newShooterConstant4;
    // }

    // newShooterConstant5 = SmartDashboard.getNumber("ShooterConstant5", 0.0);
    // if (newShooterConstant5 != previousShooterConstant5){
    //   previousShooterConstant5 = newShooterConstant5;
    // }

    // newShooterConstant6 = SmartDashboard.getNumber("ShooterConstant6", 0.0);
    // if (newShooterConstant6 != previousShooterConstant6){
    //   previousShooterConstant6 = newShooterConstant6;
    // }

    // newShooterConstant7 = SmartDashboard.getNumber("ShooterConstant7", 0.0);
    // if (newShooterConstant7 != previousShooterConstant7){
    //   previousShooterConstant7 = newShooterConstant7;
    // }
  }
}
