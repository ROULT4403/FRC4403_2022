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
  /** Constants used in Drivetrain Subsystem */
  public static final class DrivetrainConstants{
    // Ports
    // Motor Controller Ports
    public static final int portRightTop = 5;
    public static final int portRightBottom = 2; 
    public static final int portLeftTop = 6;
    public static final int portLeftBottom = 8;

    // Solenoid Ports
    public static final int[] dogShiftPort = {2, 3};
    
    // Sensor Ports
    public static final int[] kLeftEncoderPorts = {2,3};
    public static final int[] kRightEncoderPorts = {0,1};

    // Constants
    // Limiter Constants
    public static final double driveLimiter = 0.8;
    public static final double rotLimiter = 0.7; 
    
    
    // PID Constants
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Sensor Config Booleans
    public static final boolean kGyroReversed = true;
    public static final boolean kLeftEncoderReversed = true;
    public static final boolean kRightEncoderReversed = false;
    
    // Default State Booleans
    public static final boolean dogShiftDefault = true;
  }
  
  /** Constants used in Intake Subsystem */
  public static final class IntakeConstants{
    // Ports
    // Motor Controller Ports
    public static final int portIntake = 8;
    
    // Solenoid Ports
    public static final int[] intakeReleasePort = {0, 1};
    
    // Constants
    // Default State Booleans
    public static final boolean intakeReleaseDefault = false;
  }
  
  /** Constants used in Index Subsystem */
  public static final class IndexConstants {
    // Ports
    // Motor Controller Ports
    public static final int portIndex = 10;
    
    // Sensor Ports
    public static final int ultrasonicPort = 1;
  }
  
  /** Constants used in Shooter Subsystem */
  public static final class ShooterConstants {
    // Ports
    // Motor Controller Ports
    public static final int portShooterMotor = 1;
    public static final int portHoodMotor = 7;
    public static final int portTurretMotor = 4;

    // Sensor Ports
    public static final int[] hoodEncoderPorts = {0, 1};
    public static final int[] turretEncoderPorts = {2, 3};

    // Constants
    // Turret Constants
    // TODO: Change teeth
    public static final int[] turretGears= {0,1};
    public static final double turretReduction = (turretGears[0] / turretGears[1] * 360 / 2048);

    // PID Constants
    // Hood PID Constants
    public static final double hoodkP = 0;
    public static final double hoodkI = 0;
    public static final double hoodkD = 0;
    public static final double hoodkF = 0;
    
    // Turret PID Constants
    public static final double turretkP = 0;
    public static final double turretkI = 0;
    public static final double turretkD = 0;
    public static final double turretkF = 0;

    // Shooter PID Constants
    public static final double shooterkP = 0;
    public static final double shooterkI = 0;
    public static final double shooterkD = 0;
    public static final double shooterkF = 0;
  }
  
  /** Constants used RamseteCommand and Path Following */
  public static final class AutoConstants {
    // Robot Kinematics Constants
    // TODO: Verificar medidas de track width y diametro
    public static final double kTrackwidthMeters = 0.65;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 5.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Ramsete Controller Gains
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    
    // Robot Characterization and Drivetrain Feedforward Constants
    public static final double ksVolts = 1.9196;
    public static final double kvVoltSecondsPerMeter = 1.0988;
    public static final double kaVoltSecondsSquaredPerMeter = 1.2979;
    
    // RamseteController PID Gain Constants
    public static final double kPDriveVel = 3.5609;
  }
}
