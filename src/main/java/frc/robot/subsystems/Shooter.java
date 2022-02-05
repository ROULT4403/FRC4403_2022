// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  
  //Motor Controllers
  private final CANSparkMax shooterMotor = new CANSparkMax(ShooterConstants.portShooterMotor, MotorType.kBrushless);
  // private final TalonFX shooterMotor = new TalonFX(ShooterConstants.portShooterMotor);
  private final VictorSPX hoodMotor = new VictorSPX(ShooterConstants.portHoodMotor);
  private final VictorSPX turretMotor = new VictorSPX(ShooterConstants.portTurretMotor);  

  // Sensors
  // private final Encoder hoodEncoder = new Encoder(ShooterConstants.hoodEncoderPorts[0], ShooterConstants.hoodEncoderPorts[1]);
  // private final Encoder turretEncoder = new Encoder(ShooterConstants.turretEncoderPorts[0], ShooterConstants.turretEncoderPorts[1]);

  // PID Controllers
  private final PIDController hoodPID = new PIDController(ShooterConstants.hoodkP, ShooterConstants.hoodkI, 
                                                          ShooterConstants.hoodkD);
  private final PIDController turretPID = new PIDController(ShooterConstants.turretkP, ShooterConstants.turretkI, 
                                                            ShooterConstants.turretkD);

  /** Creates a new Shooter. */
  public Shooter() {

    //Configure Talon FX 
    // shooterMotor.config_kP(0, ShooterConstants.shooterkP);
    // shooterMotor.config_kI(0, ShooterConstants.shooterkI);
    // shooterMotor.config_kD(0, ShooterConstants.shooterkD);
    // shooterMotor.config_kF(0, ShooterConstants.shooterkF);
  }

  /** 
   * Uses integrated PID for shooter output
   * @param shooterSetpoint double for speed (RPM) setpoint for shooter 
    */
  public void setShooter(double shooterSetpoint){
    // shooterMotor.set(ControlMode.Velocity, shooterSetpoint * 2048 / 600);
  }

  /** 
   * Uses integrated PID for shooter output
   * @param shooterSetpoint double for speed (RPM) setpoint for shooter 
    */
  public void setShooterManual(double shooterOutput){
    shooterMotor.set(shooterOutput);
  }

  /**
    *  Returns whether controller is at reference
    */
  public boolean shootIsFinished() {
    return false;
  // return shooterMotor.isMotionProfileFinished();
}

  /** 
   * PID Control for hood output
   * @param hoodSetpoint double for hood setpoint 
   */
  public void setHood(double hoodSetpoint){
    hoodMotor.set(ControlMode.PercentOutput, hoodPID.calculate(getHoodAngle(), hoodSetpoint), DemandType.ArbitraryFeedForward, ShooterConstants.hoodkF);
  }

  /**
   * PID Control for turret output
   * @param turretSetpoint double for turret setpoint
   */
  public void setTurret(double turretSetpoint){
    turretMotor.set(ControlMode.PercentOutput, turretPID.calculate(getTurretAngle(), turretSetpoint), DemandType.ArbitraryFeedForward, ShooterConstants.turretkF);
  }

  /** 
   * Manual Hood Output
   * @param hoodSetpoint double for hood setpoint 
   */
  public void setHoodManual(double hoodOutput){
    hoodMotor.set(ControlMode.PercentOutput, hoodOutput);
  }

  /**
   * Turret manual control
   * @param turretOutput Turret output
   */
  public void setTurretManual(double turretOutput){
    turretMotor.set(ControlMode.PercentOutput, turretOutput);
  }

  /**
   * Returns whether controller is at setpoint
   * @return Boolean, true if PID is at setpoint
   */
  public boolean turretIsFinished() {
    return turretPID.atSetpoint();
  }

  /**
   * Turret manual control
   * @param direction Direction of turn, True for clockwise and False for Counter-clockwise
   */
  public void turnTurret(boolean direction){
    if(getTurretAngle() > ShooterConstants.turretACWLimit && getTurretAngle() < ShooterConstants.turretCWLimit) {
      turretMotor.set(ControlMode.PercentOutput, ShooterConstants.turretOutput * (direction ? 1 : -1));
    }
  }

  /**
   * Sweeps turret to find a target
   * @param direction Direction of initial turn, True for clockwise and False for Counter-clockwise
   */
  public void sweepTurret(boolean direction){
    boolean isComplete = false;
    if(direction) {
      if(getTurretAngle() > ShooterConstants.turretCWLimit) {
        isComplete = true;
      }
      turnTurret(!isComplete);
    } else if(!direction) {
      if(getTurretAngle() < ShooterConstants.turretACWLimit) {
        isComplete = true;
      }
      turnTurret(isComplete);
    }
  }

  /**
  *  Get turret position
  * @param 
  */
  public double getTurretAngle(){
    // return (Constants.ShooterConstants.turretGears[0] / Constants.ShooterConstants.turretGears[1] * 360 / 2048) * turretEncoder.get();
    return 20;
  }

  /**
    *  Get flywheel speed
    * @return Returns ShooterSpeed
    */
  public double getShooterSpeed(){
    return shooterMotor.getEncoder().getVelocity();
  }

  /**
    *  Get hood angle
    * @return Returns Hood Position
    */
  public double getHoodAngle(){
    // return hoodEncoder.getPosition();
    return 50;
  }

  /**
   * Manual control for hood, turret and shooter
   * @param hood double hoodOutput
   * @param turret double turretOutput
   * @param value double shooterOutput
   */
  public void testing(double hood, double turret, double value) {
    setHoodManual(hood);
    setTurretManual(turret);
    setShooterManual(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
