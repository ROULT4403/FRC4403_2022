// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexConstants;

public class Index extends SubsystemBase {

  // Motor controllers
  private final VictorSPX indexMotor = new VictorSPX(IndexConstants.portIndex);

  // Sensors
  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private final AnalogInput ultrasonic = new AnalogInput(IndexConstants.ultrasonicPort);
  private boolean detectedCargo = false;

  // private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  // private final AnalogInput ultrasonic = new AnalogInput(IndexConstants.ultrasonicPort);


  public void indexControl (double speed) {
    if(detectedCargo) {return;}  
    indexMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean hasCargo(){
    if (colorSensor.getProximity() > 50) {
      return true;
    }
    return false; 
  }

  
  @Override
  public void periodic() {
    detectedCargo = hasCargo();
  }
}