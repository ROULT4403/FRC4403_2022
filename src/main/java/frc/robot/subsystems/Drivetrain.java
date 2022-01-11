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

public class Drivetrain extends SubsystemBase {
  
  final VictorSPX topRight = new VictorSPX(DrivetrainConstants.portRightTop);
  final VictorSPX topLeft = new VictorSPX(DrivetrainConstants.portLeftTop);
  final VictorSPX bottomRight = new VictorSPX(DrivetrainConstants.portRightBottom);
  final VictorSPX bottomLeft = new VictorSPX(DrivetrainConstants.portLeftBottom);
  
  final SlewRateLimiter filterDrive = new SlewRateLimiter(10);
  final SlewRateLimiter filterRot = new SlewRateLimiter(10);


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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
