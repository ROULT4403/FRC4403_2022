// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  // Motor Controllers
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.portIntake);
  
  // Solenoids
  private final DoubleSolenoid intakeRelease = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                                                  IntakeConstants.intakeReleasePort[0], IntakeConstants.intakeReleasePort[1]);
  // private final Solenoid intakeRelease = new Solenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.intakeReleasePort);

  // Class variables
  public boolean detectedCargoIntake;
  
  // Class constants
  private boolean isReleased = IntakeConstants.intakeReleaseDefault;
  
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
  public void setIntake(double speed){
    if (true) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
    }
  }
  
  /** Toggles intake position */
  public void toggleIntakeRelease(){
    if (!isReleased) {
      intakeRelease.set(Value.kReverse);
    } else if (isReleased) {
      intakeRelease.set(Value.kForward);
    } else {
      intakeRelease.set(Value.kOff);
    }
    // intakeRelease.set(isReleased);
    isReleased = !isReleased;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("IntakeMotorOutput", intakeMotor.getMotorOutputPercent())

    Shuffleboard.getTab("Match").add("Intake Status", intakeRelease)
    .withWidget(BuiltInWidgets.kBooleanBox).withSize(1,1).withPosition(3,0);
    Shuffleboard.getTab("Match").add("IntakePneumatics", isReleased)
    .withWidget(BuiltInWidgets.kBooleanBox).withSize(1,1).withPosition(4,1);
    Shuffleboard.getTab("Match").add("IntakeTemp", intakeMotor.getTemperature())
    .withWidget(BuiltInWidgets.kTextView).withSize(1,1).withPosition(4,2);
  }
}
