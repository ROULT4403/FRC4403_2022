// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
  
  // Motor Controllers
  private final VictorSPX turretMotor = new VictorSPX(TurretConstants.portTurretMotor);  

  // Sensors
  // private final Encoder turretEncoder = new Encoder(TurretConstants.turretEncoderPorts[0], TurretConstants.turretEncoderPorts[1]);

  // PID Controllers
  private final PIDController turretPID = new PIDController(TurretConstants.turretkP, TurretConstants.turretkI, 
                                                            TurretConstants.turretkD);

  /** Creates a new Hood. */
  public Turret() {}

    /**
   * PID Control for turret output
   * @param turretSetpoint double for turret setpoint
   */
  public void setTurret(double turretSetpoint){
    turretMotor.set(ControlMode.PercentOutput, turretPID.calculate(getTurretAngle(), turretSetpoint), 
                    DemandType.ArbitraryFeedForward, TurretConstants.turretkF);
  }

    /**
   * Turret manual control
   * @param turretOutput Turret output
   */
  public void setTurretManual(double turretOutput){
    turretMotor.set(ControlMode.PercentOutput, turretOutput);
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
    // return (Constants.ShooterConstants.turretGears[0] / Constants.ShooterConstants.turretGears[1] * 360 / 2048) * turretEncoder.get();
    return 20;
  }

  /**
   * Returns whether controller is at setpoint
   * @return Boolean, true if PID is at setpoint
   */
  public boolean turretIsFinished() {
    return turretPID.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
