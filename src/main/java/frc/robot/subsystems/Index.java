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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Index extends SubsystemBase {

  // Motor controllers
  private final TalonFX indexMotor = new TalonFX(IndexConstants.portIndex);

  // Sensors
  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private boolean hasCargo = false;
  boolean isCargoAvailable = false;
  
  // Limit Switch
  private final DigitalInput indexLimitSwitch = new DigitalInput(IndexConstants.limitSwitchPort);

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
    isCargoAvailable = isCargoAvailable();
    hasCargo = hasCargo();
    if((hasCargo && isCargoAvailable) || (!hasCargo && isCargoAvailable) || (!hasCargo && !isCargoAvailable)) {
      indexMotor.set(ControlMode.PercentOutput, speed);
    } else {  
      indexMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  /**
   * Manually sets index to PercentOutput
   */
  public void setIndexManual (double speed) {
    indexMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean hasCargo(){
    return colorSensor.getProximity() > 100 ? true : false;
  }

  /**
   * Detects if cargo has crossed the intake
   * @return boolean cargo available to index
   */
  public boolean isCargoAvailable(){
    currentRots = getRotations();
    if(indexLimitSwitch.get() && currentRots < IndexConstants.indexEncoderThreshold) {return true;}
    resetRotations();
    return false;
  }

  /**
   * Returns integrated sensor rotations
   * @return double integrated sensor rotations
   */
  public double getRotations(){
    return indexMotor.getSelectedSensorPosition() / 2048;
  }

  /**
   * Sets integrated sensor rotations to 0
   */
  public void resetRotations(){
    indexMotor.setSelectedSensorPosition(0);
  }

  
  @Override
  public void periodic() {

    hasCargo = hasCargo();

    Shuffleboard.getTab("Match").add("IndexHasCargo",hasCargo())
    .withWidget(BuiltInWidgets.kBooleanBox).withSize(1,1).withPosition(6,0);
    Shuffleboard.getTab("Match").add("IndexIsAvail",isCargoAvailable())
    .withWidget(BuiltInWidgets.kBooleanBox).withSize(1,1).withPosition(6,1);
  

    SmartDashboard.putBoolean("hascargo", hasCargo());
    SmartDashboard.putBoolean("isAvail", isCargoAvailable());
    SmartDashboard.putNumber("EncRots", getRotations());

  }
}