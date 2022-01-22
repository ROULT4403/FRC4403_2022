// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

  /**
  * Uses RamseteCommand framework to follow a trajectory
  * @param s_drive Drivetrain subsystem
  * @param path Trajectory to be followed
  */
public class PathFollower extends RamseteCommand{
    private Drivetrain s_drive;
    private Trajectory path;
    public PathFollower(Drivetrain s_drive, Trajectory path){
    super(path, 
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

      this.s_drive = s_drive;
      this.path = path;

      addRequirements(s_drive);
    }
    @Override
    public void initialize(){
        s_drive.resetOdometry(path.getInitialPose());
    }
}
