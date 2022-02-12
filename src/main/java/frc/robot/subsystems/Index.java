// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexConstants;

public class Index extends SubsystemBase {

  // Motor controllers
  private final VictorSPX indexMotor = new VictorSPX(IndexConstants.portIndex);

  // Sensors
  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private boolean hasCargo = false;

  public Index() {
    // Set inverted motor
    indexMotor.setInverted(IndexConstants.indexMotorInverted);
  }

  /**
   * Set Index if no cargo is detected or cargo has gone through intake
   * @param speed
   * @param detectedCargo optional
   */
  public void setIndex (double speed, boolean... detectedCargo) {
    if(detectedCargo.length < 1) {
      detectedCargo[0] = false;
    }

    if(!hasCargo || detectedCargo[0]) {
      indexMotor.set(ControlMode.PercentOutput, speed);
    } else {  
      indexMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void setIndexManual (double speed) {
    indexMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean hasCargo(){
    return colorSensor.getProximity() > 180 ? true : false;
  }

  
  @Override
  public void periodic() {
    hasCargo = hasCargo();
    SmartDashboard.putBoolean("hascargo", hasCargo());
    SmartDashboard.putNumber("colorsensor", colorSensor.getProximity());
  }
}