// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexConstants;

public class Index extends SubsystemBase {

  // Motor controllers
  private final TalonFX indexMotor = new TalonFX(IndexConstants.portIndex);

  // Sensors
  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private boolean hasCargo = false;
  
  // Limit Switch
  private final DigitalInput indexLimitSwitch = new DigitalInput(8);

  // Index Encoder Variables
  public double currentRots;
  public double lastRots;

  public Index() {
    // Set inverted motor
    indexMotor.setInverted(IndexConstants.indexMotorInverted);
    indexMotor.setNeutralMode(NeutralMode.Brake);
    indexMotor.configFactoryDefault();
  }

  /**
   * Set Index if no cargo is detected or cargo has gone through intake
   * @param speed
   * @param detectedCargo optional
   */
  public void setIndex (double speed) {
    boolean isCargoAvailable = isCargoAvailable();
    hasCargo = hasCargo();
    if((hasCargo && isCargoAvailable) || (!hasCargo && isCargoAvailable) || (!hasCargo && !isCargoAvailable)) {
      indexMotor.set(ControlMode.PercentOutput, speed);
    } else {  
      indexMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void setIndexManual (double speed) {
    indexMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean hasCargo(){
    return colorSensor.getProximity() > 100 ? true : false;
  }

  /**
   * Detects if cargo has crossed the intake
   * @return boolean if cargo is detected
   */
  public boolean isCargoAvailable(){
    // Set current encoder status
    currentRots = getRotations();

    // Reset timer if greater than final threshold
    if (Math.abs(currentRots - lastRots) > IndexConstants.indexEncoderThreshold) {
      lastRots = 0;
    }
    
    // Start timer if current greater than threshold and integral value less than integral threshold
    if(indexLimitSwitch.get() || Math.abs(lastRots) > 0) {
      lastRots = getRotations();
      return true; 
    }

    return false;
  }

  /**
   * Returns integrated sensor rotations
   */
  public double getRotations(){
    return indexMotor.getSelectedSensorPosition() / 2048;
  }

  
  @Override
  public void periodic() {
    // hasCargo = hasCargo();
    SmartDashboard.putBoolean("hascargo", hasCargo());
  }
}