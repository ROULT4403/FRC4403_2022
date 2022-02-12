// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  // Motor Controllers
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.portIntake);
  
  // Solenoids
  private final DoubleSolenoid intakeRelease = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.intakeReleasePort[0], 
  IntakeConstants.intakeReleasePort[1]);

  // Class constants
  private boolean isReleased = IntakeConstants.intakeReleaseDefault;
  public boolean detectedCargoIntake = false;

  public double i;
  public double actualCurrent;
  public double errorCurrent;
  public double lastCurrent; 
  public double integralCurrent = 0;

  /** Susbsystem class for Drivetain, extends SubsystemBase */
  public Intake() {}
  
  /** Controls intake motor 
   * @param speed speed for intake motor in -1 to 1 range 
   * */
  public void setIntake(double speed, boolean counter){

    intakeMotor.set(ControlMode.PercentOutput, speed);

    if (counter) {
    integralCurrent = integralCurrent + errorCurrent;
    } else {
    integralCurrent = 0;
    }
  }
  
  /** Toggles intake position */
  public void toggleIntakeRelease(){
    if (!isReleased) {
      intakeRelease.set(Value.kForward);
    } else if (isReleased) {
      intakeRelease.set(Value.kReverse);
    } else {
      intakeRelease.set(Value.kOff);
    }
    isReleased = !isReleased;
  }
  
  public boolean detectCargo(){
    
    actualCurrent = Robot.pdp.getCurrent(10);
    errorCurrent = 20 - actualCurrent;

    if (i > 60) {i = 0;}

    if(actualCurrent > IntakeConstants.intakeCurrent && integralCurrent < -300) {
      i = 1;
      lastCurrent = actualCurrent;
      return false; 
    } else if (i > 0 && i < 40) {
      i++;
      return false;
    } else if (i >= 40) {
      i++;
      return true;
    }
    
    i = 0;
    lastCurrent = actualCurrent;
    return false;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    detectedCargoIntake = detectCargo();
    SmartDashboard.putBoolean("DetectedCargoIntake", detectedCargoIntake);
    SmartDashboard.putNumber("CounterVariable", i);
    SmartDashboard.putNumber("IntegralCurrent", integralCurrent);
    SmartDashboard.putNumber("IntakeFalconTemp", intakeMotor.getTemperature());
    SmartDashboard.putNumber("IntakeMotorOutput", intakeMotor.getMotorOutputPercent());

  }
}
