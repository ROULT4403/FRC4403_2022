// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class TrajectoryFollower extends CommandBase {
  public Trajectory path;
  private Drivetrain s_drive;
  private RamseteCommand ramseteCommand;
  
  /**
  * Uses RamseteCommand framework to follow a trajectory
  * @param s_drive Drivetrain subsystem
  * @param path Trajectory to be followed
  */
  public TrajectoryFollower(Drivetrain s_drive, Trajectory path) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_drive = s_drive;
    this.path = path;
    addRequirements(s_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Creates RamseteCommand istance
    ramseteCommand = new RamseteCommand(
      path, 
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
      s_drive);

    ramseteCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ramseteCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("PathStatus", "TrajectoryEnd");
    ramseteCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ramseteCommand.isFinished();
  }
}
