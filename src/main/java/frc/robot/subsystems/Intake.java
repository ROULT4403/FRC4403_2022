// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  // Motor Controllers
  private final VictorSPX intakeMotor = new VictorSPX(IntakeConstants.portIntake);
  
  // Solenoids
  private final DoubleSolenoid intakeRelease = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                                                  IntakeConstants.intakeReleasePort[0], IntakeConstants.intakeReleasePort[1]);
  
  // Class constants
  private boolean isReleased = IntakeConstants.intakeReleaseDefault;
  
  /** Susbsystem class for Drivetain, extends SubsystemBase */
  public Intake() {}
  
  /** Controls intake motor 
   * @param speed speed for intake motor in -1 to 1 range 
   * */
  public void intakeControl(double speed){
      intakeMotor.set(ControlMode.PercentOutput, speed);
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
  }
}
