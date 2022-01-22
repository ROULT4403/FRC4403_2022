// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  
  //final WPI_VictorSPX topRight = new WPI_VictorSPX(DrivetrainConstants.portRightTop);
  private final WPI_VictorSPX topLeft = new WPI_VictorSPX(DrivetrainConstants.portLeftTop);
  private final WPI_VictorSPX topRight = new WPI_VictorSPX(DrivetrainConstants.portRightTop);
  private final WPI_VictorSPX bottomLeft = new WPI_VictorSPX(DrivetrainConstants.portLeftBottom);
  private final WPI_VictorSPX bottomRight = new WPI_VictorSPX(DrivetrainConstants.portRightBottom);

  private final DifferentialDrive drive = new DifferentialDrive(topLeft, topRight);
  private final DifferentialDriveOdometry odom;

  // Sensors
  private final AHRS NavX = new AHRS(Port.kMXP);
  private final Encoder driveLeftEncoder = new Encoder(DrivetrainConstants.kLeftEncoderPorts[0], 
    DrivetrainConstants.kLeftEncoderPorts[1], DrivetrainConstants.kLeftEncoderReversed, EncodingType.k4X);
  private final Encoder driveRightEncoder = new Encoder(DrivetrainConstants.kRightEncoderPorts[0], 
    DrivetrainConstants.kRightEncoderPorts[1], DrivetrainConstants.kRightEncoderReversed, EncodingType.k4X);

  // Instantiate Pneumatics
  public final DoubleSolenoid dogShift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DrivetrainConstants.DockShiftPort[0], 
                                                             DrivetrainConstants.DockShiftPort[1]);

  // Instantiate Class Variables
  private boolean isHighGear = DrivetrainConstants.DockShiftDefault;
  // Acceleration Variables
  private double previousX = 0;
	private double dx = 0.2;
	private double previousY = 0;
	private double dy = 0.2;


    NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
    NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
  
  public Drivetrain() {

    // Setup Follower Motors
    bottomRight.follow(topRight);
    bottomLeft.follow(topLeft);

    // Setup Inverted Motors
    bottomRight.setInverted(true);
    topRight.setInverted(true);

    odom = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading())); 
    drive.setDeadband(0.05);

    driveRightEncoder.setDistancePerPulse(0.1524*Math.PI/2048);
    driveLeftEncoder.setDistancePerPulse(0.1524*Math.PI/2048);

    NavX.zeroYaw();
    resetEncoders();
  }

  public void toggleDogShift(){
    if (!isHighGear) {
      dogShift.set(Value.kForward);
    } else if (isHighGear) {
      dogShift.set(Value.kReverse);
    } else {
      dogShift.set(Value.kOff);
    }
    isHighGear = !isHighGear;
  }

  public void resetGyro(){
    NavX.zeroYaw();
  }

  public double getHeading() {
    return Math.IEEEremainder(NavX.getYaw(), 360) * (DrivetrainConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void resetEncoders(){
    driveLeftEncoder.reset();
    driveLeftEncoder.reset();
  }

  public Pose2d getPose(){
    return odom.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(driveLeftEncoder.getRate(),
                                driveRightEncoder.getRate());
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odom.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    topLeft.setVoltage(leftVolts * DrivetrainConstants.driveLimiter);
    topRight.setVoltage(rightVolts * DrivetrainConstants.driveLimiter);
    drive.feed();
  }

  public double getAverageEncoderDistance(){
    SmartDashboard.putString("Error", "Getting Distance");
    return (driveLeftEncoder.getDistance() + driveRightEncoder.getDistance()) / 2.0;
  }

  public double getTurnRate(){
    return NavX.getRate() * (DrivetrainConstants.kGyroReversed ? -1.0 : 1.0);

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block

    odom.update(Rotation2d.fromDegrees(getHeading()), driveLeftEncoder.getDistance(), driveRightEncoder.getDistance());
    
    var translation = odom.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());

    SmartDashboard.putNumber("Angle", getHeading());
    SmartDashboard.putNumber("Angle2", NavX.getYaw());
    SmartDashboard.putNumber("Distance", getAverageEncoderDistance());
    SmartDashboard.putNumber("ER", driveRightEncoder.getDistance());
    SmartDashboard.putNumber("EL", driveLeftEncoder.getDistance());
  }
}
