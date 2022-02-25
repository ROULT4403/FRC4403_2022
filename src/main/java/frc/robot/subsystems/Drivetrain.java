// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
  
  // Motor controllers
  private final CANSparkMax topLeft = new CANSparkMax(DrivetrainConstants.portLeftTop, MotorType.kBrushless);
  private final CANSparkMax topRight = new CANSparkMax(DrivetrainConstants.portRightTop, MotorType.kBrushless);
  private final CANSparkMax bottomLeft = new CANSparkMax(DrivetrainConstants.portLeftBottom, MotorType.kBrushless);
  private final CANSparkMax bottomRight = new CANSparkMax(DrivetrainConstants.portRightBottom, MotorType.kBrushless);

  // Differential Drive
  private final DifferentialDrive drive = new DifferentialDrive(topLeft, topRight);
  private final DifferentialDriveOdometry odom;

  // Sensors
  private final AHRS NavX = new AHRS(Port.kMXP);
  private final RelativeEncoder topLeftEncoder;
  private final RelativeEncoder bottomLeftEncoder;
  private final RelativeEncoder topRightEncoder;
  private final RelativeEncoder bottomRightEncoder;

  // Instantiate Pneumatics
  public final DoubleSolenoid dogShift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DrivetrainConstants.dogShiftPort[0], 
                                                            DrivetrainConstants.dogShiftPort[1]);

  // Instantiate Class Variables
  private boolean isHighGear = DrivetrainConstants.dogShiftDefault;
  // Acceleration Variables
  private double x;
  private double y;
  private double previousX = 0;
	private double dx = 0.1;
	private double previousY = 0;
	private double dy = 0.1;

  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
  
  /** Susbsystem class for Drivetain, extends SubsystemBase */
  public Drivetrain() {
    // Setup Encoders
    topLeftEncoder = topLeft.getEncoder();
    bottomLeftEncoder = bottomLeft.getEncoder();
    topRightEncoder = topRight.getEncoder();
    bottomRightEncoder = bottomLeft.getEncoder();

    // Spark Max Setup
    // Idle Mode Setup
    bottomLeft.setIdleMode(IdleMode.kCoast);
    topLeft.setIdleMode(IdleMode.kCoast);
    bottomRight.setIdleMode(IdleMode.kCoast);
    topRight.setIdleMode(IdleMode.kCoast);
    
    // Setup Follower Motors
    bottomRight.follow(topRight);
    bottomLeft.follow(topLeft);

    // Setup Inverted Motors
    bottomRight.setInverted(DrivetrainConstants.rightInverted);
    topRight.setInverted(DrivetrainConstants.rightInverted);
    
    topLeft.setInverted(DrivetrainConstants.leftInverted);
    bottomLeft.setInverted(DrivetrainConstants.leftInverted);

    // Differential Drive Setup
    odom = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading())); 
    drive.setDeadband(0.05);

    // Sensor Setup
    topLeftEncoder.setPositionConversionFactor(0.1524*Math.PI/topLeftEncoder.getCountsPerRevolution() * 0.1591);
    bottomLeftEncoder.setPositionConversionFactor(0.1524*Math.PI/topLeftEncoder.getCountsPerRevolution() * 0.1591);
    topRightEncoder.setPositionConversionFactor(0.1524*Math.PI/topLeftEncoder.getCountsPerRevolution() * 0.1591);
    bottomRightEncoder.setPositionConversionFactor(0.1524*Math.PI/topLeftEncoder.getCountsPerRevolution() * 0.1591);

    // Sensor reset
    NavX.zeroYaw();
    resetEncoders();
  }

  /**
   * Feeds Differential Drive using arcadeDrive
   * <p> Accelerates gradully with a max of 0.2 per loop
   * @param speed double for speed in -1 to 1 range
   * @param rot double for turn speed in -1 to 1 range
   */
  public void drive(double speed, double rot) {
    // Restrict Y
    y = speed * DrivetrainConstants.driveLimiter;
    if (y > previousY + dy) {
      y = previousY + dy;
    } else if (y < previousY - dy) {
      y = previousY - dy;
    }
    previousY = y;
    // Restrict X
    x = rot * DrivetrainConstants.rotLimiter;
    if (x > previousX + dx) {
      x = previousX + dx;
    } else if (x < previousX - dx) {
      x = previousX - dx;
    }
    previousX = x;

    drive.arcadeDrive(y, x);
  }

  /** Toggles the Drivetrain between high and low gear */
  /**
   * Controls DifferentialDrive in tank drive mode using voltage to take voltage sag into account
   * @param leftVolts Voltage for left side, -12 to 12
   * @param rightVolts Voltage for right side, -12 to 12
   */
  public void tankDriveVolts(double leftVolts, double rightVolts){
    topLeft.setVoltage(leftVolts * DrivetrainConstants.driveLimiter);
    topRight.setVoltage(rightVolts * DrivetrainConstants.driveLimiter);
    drive.feed();
  }

  /** Toggles dogShift to switch drivetrain gearing */
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
  
  /** Sets gyroscope yaw to 0 */
  public void resetGyro(){
    NavX.zeroYaw();
  }
  
  /** 
   * Returns gyroscope Yaw heading
   * <p>Adjusted for positive going counterclockwise
   * @return double angle heading
   */
  public double getHeading() {
    return Math.IEEEremainder(NavX.getYaw(), 360) * (DrivetrainConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /** 
   * Returns gyroscope Yaw heading
   * @return double angle heading
   */
  public double getYaw() {
    return NavX.getYaw();
  }
  
  /** Resets encoder values */
  public void resetEncoders(){
    topLeftEncoder.setPosition(0);
    bottomLeftEncoder.setPosition(0);
    topRightEncoder.setPosition(0);
    bottomRightEncoder.setPosition(0);
  }
  
  /**
   * Gets average distance for Drivetrain encoders
   * @return double
   */
  public double getAverageEncoderDistance(){
    return (getLeftEncoderPositionAverage() + getRightEncoderPositionAverage()) / 2.0;
  }
  
  /**
   * Gets average Left Drivetrain Encoder position 
   * @return double left position
   */
  public double getLeftEncoderPositionAverage() {
    return topLeftEncoder.getPosition() + bottomLeftEncoder.getPosition();
  }

  /**
   * Gets average Right Drivetrain Encoder position 
   * @return double right position
   */
  public double getRightEncoderPositionAverage() {
    return topRightEncoder.getPosition() + bottomRightEncoder.getPosition();
  }
  
  /**
   * Gets average velocity for Drivetrain encoders
   * @return double
   */
  public double getAverageEncoderVelocity(){
    return (getLeftEncoderVelocityAverage() + getRightEncoderVelocityAverage()) / 2.0;
  }

  /**
   * Gets average Left Drivetrain Encoder velocity 
   * @return double left velocity
   */
  public double getLeftEncoderVelocityAverage() {
    return topRightEncoder.getPosition() + bottomRightEncoder.getPosition();
  }

  /**
   * Gets average Right Drivetrain Encoder velocity 
   * @return double Right velocity
   */
  public double getRightEncoderVelocityAverage() {
    return topRightEncoder.getPosition() + bottomRightEncoder.getPosition();
  }
  
    /**
     * Gets drivetrain turn rate using gyroscope
     * @return double
     */
    public double getTurnRate(){
      return NavX.getRate() * (DrivetrainConstants.kGyroReversed ? -1.0 : 1.0);
    }
  
  /**
   * Gets the robots position in x and y coordinates
   * @return Pose2d
   */
  public Pose2d getPose(){
    return odom.getPoseMeters();
  }

  /**
   * Gets DdifferentialDrive wheelspeeds
   * @return DifferentialDriveWheelSpeeds
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocityAverage(),
                                            getRightEncoderVelocityAverage());
  }

  /** Resets the robot's field position
   * @param pose Pose2d new robot position
   */
  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odom.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block

    odom.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderPositionAverage(), getRightEncoderPositionAverage());
    
    var translation = odom.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());

    SmartDashboard.putNumber("GetYaw", getYaw());

    SmartDashboard.putNumber("bottomLeftTemp", bottomLeft.getMotorTemperature());
    SmartDashboard.putNumber("topLeftTemp", topLeft.getMotorTemperature());
    SmartDashboard.putNumber("bottomRightTemp", bottomRight.getMotorTemperature());
    SmartDashboard.putNumber("topRightTemp", topRight.getMotorTemperature());
  }
}