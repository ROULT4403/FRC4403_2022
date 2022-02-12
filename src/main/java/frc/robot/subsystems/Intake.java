// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.Timer;

public class Intake extends SubsystemBase {
  // Motor Controllers
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.portIntake);
  
  // Solenoids
  private final DoubleSolenoid intakeRelease = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.intakeReleasePort[0], 
  IntakeConstants.intakeReleasePort[1]);

  // Timer
  Timer intakeTimer = new Timer();

  // Class variables
  private boolean isReleased;
  public boolean detectedCargoIntake;

  public int i;
  public double actualCurrent;
  public double errorCurrent;
  public double integralCurrent;

  /** Susbsystem class for Drivetain, extends SubsystemBase */
  public Intake() {
    // Invert motor direction
    intakeMotor.setInverted(IntakeConstants.intakeMotorInverted);

    // Initialize Class Variables
    isReleased = IntakeConstants.intakeReleaseDefault;
  }
  
  /** Controls intake motor 
   * @param speed speed for intake motor in -1 to 1 range 
   * @param counter optional boolean to enable integral control
   * */
  public void setIntake(double speed, boolean... counter){

    intakeMotor.set(ControlMode.PercentOutput, speed);

    if(counter.length < 1) {return;}

    // Start integral
    if (counter[0]) {
      integralCurrent = integralCurrent + errorCurrent;
    } else {
    integralCurrent = 0;
    }
  }
  /**
   * Detects if cargo has crossed the intake
   * @return boolean if cargo is detected
   */
  public boolean hasCargo(){
    // Update current-related variables
    actualCurrent = Robot.pdp.getCurrent(10);
    errorCurrent = IntakeConstants.intakeCurrentSetpoint - actualCurrent;

    // Reset timer if greater than final threshold
    if (intakeTimer.get() > IntakeConstants.intakeTimerFinalThreshold) {
      intakeTimer.stop();
      intakeTimer.reset();
    }
    
    // Start timer if current greater than threshold and integral value less than integral threshold
    if(actualCurrent > IntakeConstants.intakeNominalCurrent && integralCurrent < IntakeConstants.intakeIntegralThreshold) {
      intakeTimer.start();
      return false; 
    // Wait until cargo settles at index
    } else if (intakeTimer.get() > 0 && intakeTimer.get() < IntakeConstants.intakeTimerInitialThreshold) {
      return false;
    // Turn index
    } else if (intakeTimer.get() >= IntakeConstants.intakeTimerInitialThreshold) {
      return true;
    }

    return false;
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    detectedCargoIntake = hasCargo();
    SmartDashboard.putBoolean("DetectedCargoIntake", detectedCargoIntake);
    SmartDashboard.putNumber("CounterVariable", i);
    SmartDashboard.putNumber("IntegralCurrent", integralCurrent);
    SmartDashboard.putNumber("IntakeFalconTemp", intakeMotor.getTemperature());
    SmartDashboard.putNumber("IntakeMotorOutput", intakeMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("IntakeTimer", intakeTimer.get());
  }
}
