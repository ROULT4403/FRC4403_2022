// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  
  //Motor Controllers
  private final TalonFX shooterMotor = new TalonFX(ShooterConstants.portShooterMotor);
  private final VictorSPX hoodMotor = new VictorSPX(ShooterConstants.portHoodMotor);
  private final VictorSPX turretMotor = new VictorSPX(ShooterConstants.portTurretMotor);

  //Sensors
  private final Encoder hoodEncoder = new Encoder(ShooterConstants.hoodEncoderPorts[0], ShooterConstants.hoodEncoderPorts[1]);
  private final Encoder turretEncoder = new Encoder(ShooterConstants.turretEncoderPorts[0], ShooterConstants.turretEncoderPorts[1]);

  //PID Controllers
  private final PIDController hoodPID = new PIDController(ShooterConstants.hoodkP, ShooterConstants.hoodkI, ShooterConstants.hoodkD);
  private final PIDController turretPID = new PIDController(ShooterConstants.turretP, ShooterConstants.turretI, ShooterConstants.turretD);


  /** Creates a new Shooter. */
  public Shooter() {

    //Configures Integrated Falcon Sensor
    shooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
  }

  /** 
   * Uses integrated PID for shooter output
   * @param shooterSetpoint double for speed (in Rpm) setpoint for shooter 
  */
  public void shoot(double shooterSetpoint){
    shooterMotor.set(ControlMode.Velocity, shooterSetpoint * 2048 / 600);
    
  }

  /** 
   * PID Control for hood output
   * @param hoodSetpoint double for hood setpoint 
   */
  public void hood(double hoodSetpoint){
    hoodMotor.set(ControlMode.PercentOutput, hoodPID.calculate(hoodEncoder.getDistance(), hoodSetpoint));

  }

  /**
   *  PID Control for turret output
   * @param turretSetpoint double for turret setpoint
   */
  public void turret(double turretSetpoint){
    turretMotor.set(ControlMode.PercentOutput, turretPID.calculate(turretEncoder.getDistance(), turretSetpoint));
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
