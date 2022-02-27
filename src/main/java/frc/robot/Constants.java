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
   * TODO: Si PCM en 12V, usar para LEDs. Si no, dejar en puente.
   * TODO: Command Group Shooter (sin tunear, solo lógica)*
   * TODO: Characterization (probar accuracy con sensor de NEO, evaluar necesidad de throug bore en drivetrain)
   * TODO: Tuneado de controladores PIDF (shooter, susan, hood)
   * TODO: Autonomos (rutas y Command Groups)
   * TODO: Definir controles (distribución driver controller, facil acceso, espacio para escalador)
   * TODO: Check if StraightDrive works correctly
   * TODO: Dashboard completo (pestaña prematch, match, postmatch)
   * TODO: Selector de Autónomo, modular
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
    public static final int[] dogShiftPort = {0, 1};

    // Constants
    // Limiter Constants
    public static final double driveLimiter = 0.8;
    public static final double rotLimiter = 0.8; 
    
    // PID Constants
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kToleranceStraightDriveAngle = 0.05;
    public static final double kToleranceStraightDriveVelocity = 0;

    // Sensor Config Booleans
    public static final boolean kGyroReversed = true;
    public static final boolean kLeftEncoderReversed = true;
    public static final boolean kRightEncoderReversed = false;
    
    // Default State Booleans
    public static final boolean dogShiftDefault = true;
    public static final boolean rightInverted = true;
    public static final boolean leftInverted = false;
  }
  
  /** Constants used in Hood Subsystem */
  public static final class HoodConstants {
    // Ports
    // Motor Controller Ports
    public static final int portHoodMotor = 6;

    // Sensor Ports
    public static final int[] hoodEncoderPorts = {5, 6};

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

    // Default State Booleans
    public static final boolean indexMotorInverted = false;
  }

  /** Constants used in Intake Subsystem */
  public static final class IntakeConstants{
    // Ports
    // Motor Controller Ports
    public static final int portIntake = 1;
    
    // Solenoid Ports
    public static final int[] intakeReleasePort = {2, 3};
    
    // Constants
    // Default State Booleans
    public static final boolean intakeReleaseDefault = false;
    public static final boolean intakeMotorInverted = false;
    
    //Intake Current Detection
    public static final int intakeNominalCurrent = 7;
    public static final int intakeCurrentSetpoint = 5;
    public static final int intakeIntegralThreshold = -300;
    
    // Intake Timer
    public static final double intakeTimerInitialThreshold = 1.3;
    public static final double intakeTimerFinalThreshold = 0.005;
  }

  
  /** Constants used in Shooter Subsystem */
  public static final class ShooterConstants {

    public static final int relayPort = 0;

    // Ports
    // Motor Controller Ports
    public static final int portShooterMotor = 0;
    
    // Constants
    // PID Constants
    public static final double shooterkP = 0.11846;
    public static final double shooterkI = 0.00014;
    public static final double shooterkD = 0;
    public static final double shooterkF = 0.0558;


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
    public static final int[] turretEncoderPorts = {0,4};

    // Constants
    public static final int[] turretGears = {30,260};
    // public static final float turretReduction = (TurretConstants.turretGears[0] * 360) / (TurretConstants.turretGears[1] * 2048);
    public static final double turretReduction = 0.0202824519230769;
    public static final int turretCWLimit = 180;
    public static final int turretACWLimit = -160;
        
    // Output Constants
    public static final double turretOutput = -0.5;

    // PID Constants
    public static final double turretkP = 0.03;
    public static final double turretkI = 0.01;
    public static final double turretkD = 0.003;
    public static final double turretkF = 0;

    // Default State Booleans
    public static final boolean turretMotorInverted = true;
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
