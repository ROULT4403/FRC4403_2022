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
  public static final class DrivetrainConstants{
    
        public static final int portRightTop = 5;
        public static final int portRightBottom = 2; 
        public static final int portLeftTop = 6;
        public static final int portLeftBottom = 8;

        public static final double driveLimiter = 0.8;
        public static final double rotLimiter = 0.7; 
        
        // TODO: Verificar medidas de track width y diametro
        public static final double kTrackwidthMeters = 0.65;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
        
        public static final boolean kGyroReversed = true;
        
        public static final int[] dogShiftPort = {2, 3};
        public static final boolean dogShiftDefault = false;

        public static final int[] kLeftEncoderPorts = {2,3};
        public static final int[] kRightEncoderPorts = {0,1};
        public static final boolean kLeftEncoderReversed = true;
        public static final boolean kRightEncoderReversed = false;

        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 1.9196;
        public static final double kvVoltSecondsPerMeter = 1.0988;
        public static final double kaVoltSecondsSquaredPerMeter = 1.2979;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 3.5609;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static final class IntakeConstants{
        public static final int portIntake = 4;

        public static final int[] intakeReleasePort = {0, 1};
        public static final boolean intakeReleaseDefault = false;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 5.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    
      // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
  
      }
  
  
  
  
}
