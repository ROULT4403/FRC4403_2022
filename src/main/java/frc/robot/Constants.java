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
    public static final int portRightTop = 2;
    public static final int portRightBottom = 3; 
    public static final int portLeftTop = 10;
    public static final int portLeftBottom = 6;

    // Solenoid Ports
    public static final int[] dogShiftPort = {0, 1};
    
    // Sensor Ports
    public static final int[] kLeftEncoderPorts = {2,3};
    public static final int[] kRightEncoderPorts = {4,5};

    // Constants
    // Limiter Constants
    public static final double driveLimiter = 1;
    public static final double rotLimiter = 1; 
    
    
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
    public static final int portIntake = 9;
    
    // Solenoid Ports
    public static final int[] intakeReleasePort = {2, 3};
    
    // Constants
    // Default State Booleans
    //Intake Voltage
    public static final int intakeCurrent = 26;
    public static final boolean intakeReleaseDefault = false;
  }
  
  /** Constants used in Index Subsystem */
  public static final class IndexConstants {
    // Ports
    // Motor Controller Ports
    public static final int portIndex = 5;
    
    // Sensor Ports
    public static final int ultrasonicPort = 1;
  }
  
  /** Constants used in Shooter Subsystem */
  public static final class ShooterConstants {
    // Ports
    // Motor Controller Ports
    public static final int portShooterMotor = 3;
 
    // Constants
    // PID Constants
    public static final double shooterkP = 0.5;
    public static final double shooterkI = 0;
    public static final double shooterkD = 5;
    public static final double shooterkF = 0.07;
  }

  /** Constants used in Hood Subsystem */
  public static final class HoodConstants {
    // Ports
    // Motor Controller Ports
    public static final int portHoodMotor = 7;

    // Sensor Ports
    public static final int[] hoodEncoderPorts = {0, 1};

    // Output Constants
    public static final double hoodOutput = 0.2;

    // PID Constants
    public static final double hoodkP = 0;
    public static final double hoodkI = 0;
    public static final double hoodkD = 0;
    public static final double hoodkF = 0;
  }

  /** Constants used in Turret Subsystem */
  public static final class TurretConstants {
    // Ports
    // Motor Controller Ports
    public static final int portTurretMotor = 4;

    // Sensor Ports
    public static final int[] turretEncoderPorts = {2, 3};

    // Constants
    public static final int[] turretGears= {30,260};
    public static final double turretReduction = (turretGears[0] / turretGears[1] * 360 / 2048);
    public static final int turretCWLimit = 100;
    public static final int turretACWLimit = 10;
        
    // Output Constants
    public static final double turretOutput = 0.2;

    // PID Constants
    public static final double turretkP = 0;
    public static final double turretkI = 0;
    public static final double turretkD = 0;
    public static final double turretkF = 0;
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
