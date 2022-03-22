// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /** 
   * * Este orden no es opcional, si desean cambiarlo hablenlo conmigo primero
   * TODO: Autos con Rutas
   * TODO: Selector de Autónomo
  */
  
  /** Constants used in Drivetrain Subsystem */
  public static final class DrivetrainConstants{
    // Ports
    // Motor Controller Ports
    public static final int portRightTop = 1;
    public static final int portRightBottom = 2; 
    public static final int portLeftTop = 4;
    public static final int portLeftBottom = 3;

    // Solenoid Ports
    public static final int[] dogShiftPort = {3, 4};
    // public static final int dogShiftPort = 0;

    // Constants
    // Limiter Constants
    public static final double driveLimiter = 0.8;
    public static final double rotLimiter = 0.8; 
    
    // PID Constants
    public static final double kP = 1.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kToleranceStraightDriveAngle = 0.05;
    public static final double kToleranceStraightDriveVelocity = 0;

    public static final double kToleranceDriveDistance = 0.2;
    public static final double kToleranceTurnAngle = 1;

    // Sensor Config Booleans
    public static final boolean kGyroReversed = true;
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;
    
    // Default State Booleans
    public static final boolean dogShiftDefault = true;
    public static final boolean rightInverted = false;
    public static final boolean leftInverted = true;
  }
  
  /** Constants used in Hood Subsystem */
  public static final class HoodConstants {
    // Ports
    // Motor Controller Ports
    public static final int portHoodMotor = 6;

    // Sensor Ports
    public static final int[] hoodEncoderPorts = {5, 4};
    public static final int limitSwitchPort = 0;

    // Output Constants
    public static final double hoodOutput = 0.5;

    // PID Constants
    public static final double hoodkP = 0.05;
    public static final double hoodkI = 0;
    public static final double hoodkD = 0;
    public static final double hoodkF = 0;

    // Default State Booleans
    public static final boolean hoodMotorInverted = true;
  }

  /** Constants used in Index Subsystem */
  public static final class IndexConstants {
    // Ports
    // Motor Controller Ports
    public static final int portIndex = 5;
    
    // Sensor Ports
    public static final int ultrasonicPort = 1;
    public static final int limitSwitchPort = 8;

    // Default State Booleans
    public static final boolean indexMotorInverted = true;

    // Index Rotation Limits
    public static final double indexEncoderThreshold = 0.02;
  }

  /** Constants used in Intake Subsystem */
  public static final class IntakeConstants{
    // Ports
    // Motor Controller Ports
    public static final int portIntake = 1;
    
    // Solenoid Ports
    public static final int[] intakeReleasePort = {7, 1};
    // public static final int intakeReleasePort = 1;

    // Constants
    // Default State Booleans
    public static final boolean intakeReleaseDefault = true;
    public static final boolean intakeMotorInverted = false;
    
    //Intake Current Detection
    public static final int intakeNominalCurrent = 7;
  }

  
  /** Constants used in Shooter Subsystem */
  public static final class ShooterConstants {
    // Ports
    // Motor Controller Ports
    public static final int portShooterMotor = 0;
    
    // Constants
    // PID Constants
    public static final double shooterkP = 1.1;
    public static final double shooterkI = 0;
    public static final double shooterkD = 0;
    public static final double shooterkF = 0.0475;


    // Default State Booleans
    public static final boolean shooterMotorInverted = false;
    public static final boolean relayDefault = false;
  }

  /** Constants used in Turret Subsystem */
  public static final class TurretConstants {
    // Ports
    // Motor Controller Ports
    public static final int portTurretMotor = 8;

    // Sensor Ports
    public static final int[] turretEncoderPorts = {2,1};

    // Constants
    public static final int[] turretGears = {30,260};
    // public static final float turretReduction = (TurretConstants.turretGears[0] * 360) / (TurretConstants.turretGears[1] * 2048);
    public static final double turretReduction = 0.0202824519230769;
    public static final int turretCWLimit = 100;
    public static final int turretACWLimit = -100;
        
    // Output Constants
    public static final double turretOutput = -0.7;

    // PID Constants
    public static final double turretkP = 0.03;
    public static final double turretkI = 0.01;
    public static final double turretkD = 0.003;
    public static final double turretkF = 0;

    // Default State Booleans
    public static final boolean turretMotorInverted = true;
  }
  
  public static final class ClimberConstants{
  /** Constants used in Climber subsystem */
  //Ports
  //Motor Controller Ports
  public static final int portClimber = 9;

  //Solenoid Ports
  public static final int[] climberSolenoidPorts = {5,2};
  public static final int[] climberSolenoidPortsB = {4,3};

  //Constants
  //Default State Booleans
  public static final boolean climberLeftInverted = true;
  public static final boolean climberRightInverted = false;
  public static final boolean climberAttatchedDefault = true;
  public static final boolean climberExtendedDefault = false;

  //Treshold
  public static final int climberMinPositioon = 200;
  public static final int climberMaxPos = 30000;
  }

  /** Constants used RamseteCommand and Path Following */
  public static final class AutoConstants {
    // Robot Kinematics Constants
    public static final double kTrackwidthMeters = 0.61874;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 5.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Ramsete Controller Gains
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    
    // Robot Characterization and Drivetrain Feedforward Constants
    //public static final double ksVolts = 0.24004;
    //public static final double kvVoltSecondsPerMeter = 3.287;
    //public static final double kaVoltSecondsSquaredPerMeter = 0.59243;
    
    // RamseteController PID Gain Constants
    //public static final double kPDriveVel = 4.4911;

    public static final double ksVolts = 0.23838;
    public static final double kvVoltSecondsPerMeter = 3.2883;
    public static final double kaVoltSecondsSquaredPerMeter = 0.53537;
    
    // RamseteController PID Gain Constants
    public static final double kPDriveVel = 4.4206;

    //Path choosing variable
    public static final String pathChoose = "Auto1.1.2";

    //Backup Constants
    public static final double autoDistance = 2;
    public static final double autoAngle = 90;

    public static final double autoTwoDistance = 0.7;
    public static final double autoTwoAngle = 110;
  }
}
