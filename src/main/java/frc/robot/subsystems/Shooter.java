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

  // FPID Controllers
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
  public void shoot(double shooterSetpoint){
    // shooterMotor.set(ControlMode.Velocity, shooterSetpoint * 2048 / 600);
    shooterMotor.set(shooterSetpoint);
    
  }

  /**
    *  Returns whether controller is at reference
    * @param
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
    // hoodMotor.set(ControlMode.PercentOutput, hoodPID.calculate(hoodEncoder.getDistance(), hoodSetpoint), DemandType.ArbitraryFeedForward, ShooterConstants.hoodkF);
    hoodMotor.set(ControlMode.PercentOutput, hoodSetpoint);
  }

  /**
   *  PID Control for turret output
   * @param turretSetpoint double for turret setpoint
   */
  public void setTurret(double turretSetpoint){
    // turretMotor.set(ControlMode.PercentOutput, turretPID.calculate(getTurretAngle(), turretSetpoint), DemandType.ArbitraryFeedForward, ShooterConstants.turretkF);
    turretMotor.set(ControlMode.PercentOutput, turretSetpoint);
  }

  /**
   * Returns whether controller is at setpoint
   * @param
   */
  public boolean turretIsFinished() {
    // return turretPID.atSetpoint();
    return false;
  }

  /**
   * Turret manual control
   * TODO: Revisar lógica
   * @param direction Direction of turn, True for clockwise and False for Counter-clockwise
   */
  public void turretManual(boolean direction){

    if(getTurretAngle() > 10 && getTurretAngle() < 10) {
      turretMotor.set(ControlMode.PercentOutput, 0.7 * (direction ? 1 : -1));
    }
  }

  /**
   * Sweeps turret
   * @param direction Direction of turn, True for clockwise and False for Counter-clockwise
   * @return
   */
  public void sweepTurret(boolean direction){
    double dir = 1;

    if (direction) {
      dir = 1;

      while (getTurretAngle() < 100){
        turretMotor.set(ControlMode.PercentOutput, 0.7 * dir);
      }
      while (getTurretAngle() > 5){
        turretMotor.set(ControlMode.PercentOutput, -0.7 * dir);
      }
    } else {
      dir = -1;

      while (getTurretAngle() < 100){
        turretMotor.set(ControlMode.PercentOutput, 0.7 * dir);
      }
      while (getTurretAngle() > 5){
        turretMotor.set(ControlMode.PercentOutput, -0.7 * dir);
      }
    }

  }

    /**
      *  Get turret angle
      * @param 
      */
    public double getTurretAngle(){
      return 0;
      // return (Constants.ShooterConstants.turretGears[0] / Constants.ShooterConstants.turretGears[1] * 360 / 2048) * turretEncoder.get();
    }

    /**
      *  Get flywheel speed
      * @param 
      */
    public double getShootSpeed(){
      return 10;
    }

    /**
      *  Get hood angle
      * @param 
      */
    public double getHoodAngle(){
      return 10;
    }

    public void testing(double hood, double turret, double value) {
      this.setHood(hood);
      this.setTurret(turret);
      shoot(value);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
