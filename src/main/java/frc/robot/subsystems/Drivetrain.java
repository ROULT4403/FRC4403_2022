// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Drivetrain extends SubsystemBase {
  
  public final VictorSPX topRight = new VictorSPX(DrivetrainConstants.portRightTop);
  public final VictorSPX topLeft = new VictorSPX(DrivetrainConstants.portLeftTop);
  public final VictorSPX bottomRight = new VictorSPX(DrivetrainConstants.portRightBottom);
  public final VictorSPX bottomLeft = new VictorSPX(DrivetrainConstants.portLeftBottom);
  
  public final SlewRateLimiter filterDrive = new SlewRateLimiter(10);
  public final SlewRateLimiter filterRot = new SlewRateLimiter(10);

  public final DoubleSolenoid DockShift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DrivetrainConstants.DockShiftPort[0], DrivetrainConstants.DockShiftPort[1]);

  private boolean shift = DrivetrainConstants.DockShiftDefault;

  public Drivetrain() {

    bottomRight.follow(topRight);
    bottomLeft.follow(topLeft);

    bottomRight.setInverted(true);
    topRight.setInverted(true);

  }
  public void drive (double velocidad, double velocidadRot) {
    double filterSpeed = filterDrive.calculate(velocidad);
    double filterTurn = filterRot.calculate(velocidadRot);

    topLeft.set(ControlMode.PercentOutput, DrivetrainConstants.driveLimiter * filterSpeed, DemandType.ArbitraryFeedForward, filterTurn * DrivetrainConstants.rotLimiter);
    topRight.set(ControlMode.PercentOutput, DrivetrainConstants.driveLimiter * filterSpeed, DemandType.ArbitraryFeedForward, -filterTurn * DrivetrainConstants.rotLimiter);

  }
  public void DockShift() {
    if (!shift) {
      DockShift.set(Value.kForward);
    } else if (shift) {
      DockShift.set(Value.kReverse);
    } else {
      DockShift.set(Value.kOff);
    }
    shift = !shift;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
