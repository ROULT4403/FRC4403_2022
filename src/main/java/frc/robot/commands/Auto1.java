// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto1 extends SequentialCommandGroup {
  /** Creates a new Auto1. */
  public Auto1() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Drivetrain s_drive = new Drivetrain();
    addCommands(
      new RamseteCommand(Robot.patth, 
      s_drive::getPose, 
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), 
      new SimpleMotorFeedforward(DrivetrainConstants.ksVolts,
                                    DrivetrainConstants.kvVoltSecondsPerMeter,
                                    DrivetrainConstants.kaVoltSecondsSquaredPerMeter), 
      DrivetrainConstants.kDriveKinematics, 
      s_drive::getWheelSpeeds, 
      new PIDController(DrivetrainConstants.kPDriveVel, 0, 0), 
      new PIDController(DrivetrainConstants.kPDriveVel, 0, 0), 
      s_drive::tankDriveVolts, 
      s_drive),
      new RamseteCommand(Robot.patth2, 
      s_drive::getPose, 
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), 
      new SimpleMotorFeedforward(DrivetrainConstants.ksVolts,
                                    DrivetrainConstants.kvVoltSecondsPerMeter,
                                    DrivetrainConstants.kaVoltSecondsSquaredPerMeter), 
      DrivetrainConstants.kDriveKinematics, 
      s_drive::getWheelSpeeds, 
      new PIDController(DrivetrainConstants.kPDriveVel, 0, 0), 
      new PIDController(DrivetrainConstants.kPDriveVel, 0, 0), 
      s_drive::tankDriveVolts, 
      s_drive)
      );
  }
}
