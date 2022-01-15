// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
  
  //final WPI_VictorSPX topRight = new WPI_VictorSPX(DrivetrainConstants.portRightTop);
  final WPI_VictorSPX topLeft = new WPI_VictorSPX(DrivetrainConstants.portLeftTop);
  final WPI_VictorSPX bottomRight = new WPI_VictorSPX(DrivetrainConstants.portRightBottom);
  final WPI_VictorSPX bottomLeft = new WPI_VictorSPX(DrivetrainConstants.portLeftBottom);

  final WPI_VictorSPX topRight = new WPI_VictorSPX(DrivetrainConstants.portRightTop);
  
  public static DifferentialDrive Drive = new DifferentialDrive(topLeft, topRight);

  
  // Sensors
  private final AHRS NavX = new AHRS(Port.kMXP);

  final SlewRateLimiter filterDrive = new SlewRateLimiter(10);
  final SlewRateLimiter filterRot = new SlewRateLimiter(10);

  private final DifferentialDriveOdometry odom;
  
  public Drivetrain() {

    bottomRight.follow(topRight);
    bottomLeft.follow(topLeft);

    bottomRight.setInverted(true);
    topRight.setInverted(true);

    odom = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));  

  }
  public void drive(double velocidad, double velocidadRot) {

    //topLeft.set(ControlMode.PercentOutput, DrivetrainConstants.driveLimiter * filterSpeed, DemandType.ArbitraryFeedForward, filterTurn * DrivetrainConstants.rotLimiter);
    //topRight.set(ControlMode.PercentOutput, DrivetrainConstants.driveLimiter * filterSpeed, DemandType.ArbitraryFeedForward, -filterTurn * DrivetrainConstants.rotLimiter);

    drive.arcadeDrive(velocidad, velocidadRot);
  }

  public double getHeading() {
    return Math.IEEEremainder(NavX.getYaw(), 360) * (DrivetrainConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void resetEncoders(){
    topLeft.setSelectedSensorPosition(0);
    topRight.setSelectedSensorPosition(0);
  }

  public Pose2d getPose(){
    return odom.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(topRight.getSelectedSensorVelocity(),
                                topRight.getSelectedSensorVelocity());
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odom.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    topLeft.setVoltage(leftVolts);
    topRight.setVoltage(rightVolts);
    Drive.feed();
  }

  public double getAverageEncoderDistance(){
    return (topLeft.getSelectedSensorPosition() + topRight.getSelectedSensorPosition()) / 2.0;
  }

  public double getTurnRate(){
    return NavX.getRate()* (DrivetrainConstants.kGyroReversed ? -1.0 : 1.0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     // Update the odometry in the periodic block
     odom.update(Rotation2d.fromDegrees(getHeading()), topLeft.getSelectedSensorPosition(),
     topRight.getSelectedSensorPosition());
  }
}
