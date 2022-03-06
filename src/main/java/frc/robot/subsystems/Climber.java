// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  //Motor Controllers
  public TalonFX climberLeft = new TalonFX(ClimberConstants.portClimberLeft);
  public TalonFX climberRight = new TalonFX(ClimberConstants.portClimberRight);

  //Solenoids
  public DoubleSolenoid climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                                            ClimberConstants.climberSolenoidPorts[0], ClimberConstants.climberSolenoidPorts[1]);

  //Class constants
  public boolean isAttatched = ClimberConstants.climberAttatchedDefault;

  public Climber() {
    //Setup Follower Motors
    climberRight.follow(climberLeft);

    //Setup Inverted Motors
    climberLeft.setInverted(ClimberConstants.climberLeftInverted);
    climberRight.setInverted(ClimberConstants.climberRightInverted);

  }
  /**
   * Set climber output for upwards motion
   * @param speed double for speed in a -1 to 1 range
   */
  public void setAltitude(double speed){
    if( climberLeft.getSelectedSensorPosition() < ClimberConstants.climberTreshold){
    climberLeft.set(ControlMode.PercentOutput, Math.abs(speed));
    }else{
      climberLeft.set(ControlMode.PercentOutput, speed);
    }
  }

  public void climberAttatch(){
    if (!isAttatched) {
      climberSolenoid.set(Value.kReverse);
    } else if (isAttatched) {
      climberSolenoid.set(Value.kForward);
    } else {
      climberSolenoid.set(Value.kOff);
    }

    isAttatched = !isAttatched;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
