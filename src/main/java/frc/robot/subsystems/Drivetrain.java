// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain extends SubsystemBase {
  
  public final WPI_VictorSPX topRight = new WPI_VictorSPX(DrivetrainConstants.portRightTop);
  public final WPI_VictorSPX topLeft = new WPI_VictorSPX(DrivetrainConstants.portLeftTop);
  public final WPI_VictorSPX bottomRight = new WPI_VictorSPX(DrivetrainConstants.portRightBottom);
  public final WPI_VictorSPX bottomLeft = new WPI_VictorSPX(DrivetrainConstants.portLeftBottom);
  
  public final SlewRateLimiter filterDrive = new SlewRateLimiter(10);
  public final SlewRateLimiter filterRot = new SlewRateLimiter(10);

  public final DoubleSolenoid DockShift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DrivetrainConstants.DockShiftPort[0], DrivetrainConstants.DockShiftPort[1]);
  public final DifferentialDrive drive = new DifferentialDrive(topLeft, topRight);

  public final AHRS navx = new AHRS(Port.kMXP);

  //public final Encoder encoderLeft = new Encoder(DrivetrainConstants.EncoderLeftPort[0], DrivetrainConstants.EncoderLeftPort[1], false, EncodingType.k4X);
  //public final Encoder encoderRight = new Encoder(DrivetrainConstants.EncoderRightPort[0], DrivetrainConstants.EncoderRightPort[1], true, EncodingType.k4X);

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

    //topLeft.set(ControlMode.PercentOutput, DrivetrainConstants.driveLimiter * filterSpeed, DemandType.ArbitraryFeedForward, filterTurn * DrivetrainConstants.rotLimiter);
    //topRight.set(ControlMode.PercentOutput, DrivetrainConstants.driveLimiter * filterSpeed, DemandType.ArbitraryFeedForward, -filterTurn * DrivetrainConstants.rotLimiter);

    drive.arcadeDrive(filterSpeed, filterTurn);
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
