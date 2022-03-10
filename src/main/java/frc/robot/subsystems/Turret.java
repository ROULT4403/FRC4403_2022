// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
  
  // Motor Controllers
  private final VictorSPX turretMotor = new VictorSPX(TurretConstants.portTurretMotor);  

  // Sensors
  private final Encoder turretEncoder = new Encoder(TurretConstants.turretEncoderPorts[0], TurretConstants.turretEncoderPorts[1], false , EncodingType.k4X);

  // PID Controllers
  private final PIDController turretPID = new PIDController(TurretConstants.turretkP, TurretConstants.turretkI, 
                                                            TurretConstants.turretkD);
  private double PIDoutput;

  // PID variables
  double newKP;
  double previousKP;
  double newKI;
  double previousKI;
  double newKD;
  double previousKD;

  /** Creates a new Hood. */
  public Turret() {
    // Set inverted motor
    turretMotor.setInverted(TurretConstants.turretMotorInverted);

    // turretEncoder.setDistancePerPulse(TurretConstants.turretReduction);
    SmartDashboard.putNumber("kP", newKP);
    SmartDashboard.putNumber("kI", newKI);
    SmartDashboard.putNumber("kD", newKD);
  }

    /**
   * PID Control for turret output
   * @param turretSetpoint double for turret setpoint
   */
  public void setTurret(double turretSetpoint){
    PIDoutput = turretPID.calculate(getTurretAngle(), turretSetpoint);

    if (Robot.tV) {
    if (getTurretAngle() < TurretConstants.turretCWLimit && getTurretAngle() > -TurretConstants.turretCWLimit) {
      turretMotor.set(ControlMode.PercentOutput, PIDoutput, 
      DemandType.ArbitraryFeedForward, TurretConstants.turretkF);
    } else {
      turretMotor.set(ControlMode.PercentOutput, 0);
    }
  }
  }

    /**
   * Turret manual control
   * @param turretOutput Turret output
   */
  public void setTurretManual(double turretOutput){
    if (getTurretAngle() < TurretConstants.turretCWLimit && turretOutput > 0) {
      turretMotor.set(ControlMode.PercentOutput, turretOutput);
    } else if (getTurretAngle() > TurretConstants.turretACWLimit && turretOutput < 0) {
      turretMotor.set(ControlMode.PercentOutput, turretOutput);
    } else {
      turretMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  /**
   * Turret manual control
   * @param direction Direction of turn, True for clockwise and False for Counter-clockwise
   */
  public void turnTurret(boolean direction){
    if(getTurretAngle() > TurretConstants.turretACWLimit && getTurretAngle() < TurretConstants.turretCWLimit) {
      turretMotor.set(ControlMode.PercentOutput, TurretConstants.turretOutput * (direction ? 1 : -1));
    }
  }
  
  /**
   * Sweeps turret to find a target
   * @param direction Direction of initial turn, True for clockwise and False for Counter-clockwise
   */
  public void sweepTurret(boolean direction){
    boolean isComplete = false;
    if(direction) {
      if(getTurretAngle() > TurretConstants.turretCWLimit) {
        isComplete = true;
      }
      turnTurret(!isComplete);
    } else if(!direction) {
      if(getTurretAngle() < TurretConstants.turretACWLimit) {
        isComplete = true;
      }
      turnTurret(isComplete);
    }
  }
  
  /**
   *  Get turret position
   * @return double turret position
   */
  public double getTurretAngle(){
    // return 360 / 260 * 30 / 2048 * turretEncoder.get();
    return turretEncoder.get() * TurretConstants.turretReduction;
  }

  /**
   * Returns whether controller is at setpoint
   * @return Boolean, true if PID is at setpoint
   */
  public boolean turretIsFinished() {
    return turretPID.atSetpoint();
  }

  public void resetAngle(){
    turretEncoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("TurretAngle", getTurretAngle());
    SmartDashboard.putBoolean("turretIsFinished", turretIsFinished());

    newKP = SmartDashboard.getNumber("kP", 0.0);
    if (newKP != previousKP){
      turretPID.setP(newKP);
      previousKP = newKP;
    }
    
    newKI = SmartDashboard.getNumber("kI", 0.0);
    if (newKI != previousKI){
      turretPID.setI(newKI);
      previousKI = newKI;
    }

    newKD = SmartDashboard.getNumber("kD", 0.0);
    if (newKD != previousKD){
      turretPID.setD(newKD);
      previousKD = newKD;
    }
  }
}