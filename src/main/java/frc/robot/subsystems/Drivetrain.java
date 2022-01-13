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
  
  // Instantiate Motor Controllers
  public final WPI_VictorSPX topRight = new WPI_VictorSPX(DrivetrainConstants.portRightTop);
  public final WPI_VictorSPX topLeft = new WPI_VictorSPX(DrivetrainConstants.portLeftTop);
  public final WPI_VictorSPX bottomRight = new WPI_VictorSPX(DrivetrainConstants.portRightBottom);
  public final WPI_VictorSPX bottomLeft = new WPI_VictorSPX(DrivetrainConstants.portLeftBottom);
  
  // Instantiate DifferentialDrive
  public final DifferentialDrive drive = new DifferentialDrive(topLeft, topRight);

  // Instantiate Acceleration Filters
  public final SlewRateLimiter filterDrive = new SlewRateLimiter(0.5);
  public final SlewRateLimiter filterRot = new SlewRateLimiter(10);

  // Instantiate Pneumatics
  public final DoubleSolenoid DockShift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DrivetrainConstants.DockShiftPort[0], DrivetrainConstants.DockShiftPort[1]);

  // Acceleration Variables
  private double previousX = 0;
	private double dx = 0.2;

	private double previousY = 0;
	private double dy = 0.2;

  // Instantiate Sensors
  public final AHRS navx = new AHRS(Port.kMXP); // NavX Gyro
  // public final Encoder encoderLeft = new Encoder(DrivetrainConstants.EncoderLeftPort[0], DrivetrainConstants.EncoderLeftPort[1], 
  //                                                false, EncodingType.k4X);
  // public final Encoder encoderRight = new Encoder(DrivetrainConstants.EncoderRightPort[0], DrivetrainConstants.EncoderRightPort[1], 
  //                                                 true, EncodingType.k4X);


  // Instantiate Class Variables
  private boolean shift = DrivetrainConstants.DockShiftDefault;

  public Drivetrain() {

    // Setup Follower Motors
    bottomRight.follow(topRight);
    bottomLeft.follow(topLeft);

    // Setup Inverted Motors
    bottomRight.setInverted(true);
    topRight.setInverted(true);

    drive.setDeadband(0.05);

  }
  public void drive (double speed, double rot) {
    double y = speed * DrivetrainConstants.driveLimiter;
    if (y > previousY + dy) {
      y = previousY + dy;
    } else if (y < previousY - dy) {
      y = previousY - dy;
    }
    previousY = y;
    // Restrict X
    double x = rot * DrivetrainConstants.rotLimiter;
    if (x > previousX + dx) {
      x = previousX + dx;
    } else if (x < previousX - dx) {
      x = previousX - dx;
    }
    previousX = x;

    drive.arcadeDrive(y, x);
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
