// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.LineFollower;
import frc.robot.subsystems.Drivetrain;

public class Auto1 extends SequentialCommandGroup {
  
  /** Follows two trajectories
   * @param s_drive Drivetrain subsystem
   */
  public Auto1(Drivetrain s_drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LineFollower(s_drive , Robot.path),
      new LineFollower(s_drive , Robot.path2).andThen(() -> s_drive.tankDriveVolts(0,0))
      );
  }
}