// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  //Motor Controllers
  public TalonFX climberMotor = new TalonFX(ClimberConstants.portClimber);

  //Solenoids
  public DoubleSolenoid climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                                            ClimberConstants.climberSolenoidPorts[0], ClimberConstants.climberSolenoidPorts[1]);
  
  // Limit switch
  private final DigitalInput climberLimitSwitch = new DigitalInput(6);

  //Class constants
  public boolean isFlexed = ClimberConstants.climberAttatchedDefault;

  public Climber() {
    // Set Break Mode
    climberMotor.setNeutralMode(NeutralMode.Brake);

    //Setup Inverted Motors
    climberMotor.setInverted(ClimberConstants.climberMotorInverted);

    resetPosition();
  }

  public void setArmsManual(double speed) {
    climberMotor.set(ControlMode.PercentOutput, speed);
    resetPosition();
  }

  /**
   * Set climber output for upwards motion
   * @param speed double for speed in a -1 to 1 range
   */
  public void setAltitude(double speed){
    if (climberMotor.getSelectedSensorPosition() > ClimberConstants.climberMaxPos && speed > 0){
      climberMotor.set(ControlMode.PercentOutput, 0);
    } else if (climberLimitSwitch.get() && speed < 0){
      climberMotor.set(ControlMode.PercentOutput, 0);
    } else {
      climberMotor.set(ControlMode.PercentOutput, speed);
    }
    resetPosition();
  }

  public void climberFlex(){
    if (!isFlexed) {
      climberSolenoid.set(Value.kReverse);
    } else if (isFlexed) {
      climberSolenoid.set(Value.kForward);
    } else {
      climberSolenoid.set(Value.kOff);
    }
    isFlexed = !isFlexed;
  }

  /**
 * Move climber until Limit Switch is triggered, then reset angle
 */
  public void resetPosition() {
    if (climberLimitSwitch.get()) {
      climberMotor.setSelectedSensorPosition(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClimberEncoder", climberMotor.getSelectedSensorPosition());
    SmartDashboard.putBoolean("ClimberLimitSwitch", climberLimitSwitch.get());
  }
}
