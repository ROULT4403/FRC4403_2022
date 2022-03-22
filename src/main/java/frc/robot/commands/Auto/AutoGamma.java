// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.Shoot;
import frc.robot.commands.TrajectoryFollower;
import frc.robot.subsystems.*;

public class AutoGamma extends SequentialCommandGroup {
  /** Creates a new OneTrajectoryAuto. */
  public AutoGamma(Trajectory pathOne, Trajectory pathTwo, Drivetrain s_drive, Hood s_hood, Index s_index, Intake s_intake, Shooter s_shooter,  Turret s_turret) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Shoot(s_index, s_shooter, s_hood, s_turret, s_shooter.getShooterTargetSpeed(), 
                s_hood.getHoodTargetAngle(), Robot.tX + s_turret.getTurretAngle()),
      new TrajectoryFollower(s_drive, pathOne)
      .alongWith(new RunCommand(() -> s_intake.setIntake(0.3), s_intake)),
      new TrajectoryFollower(s_drive, pathOne)
      .alongWith(new RunCommand(() -> s_intake.setIntake(0.3), s_intake)),
      new Shoot(s_index, s_shooter, s_hood, s_turret, s_shooter.getShooterTargetSpeed(), 
                s_hood.getHoodTargetAngle(), Robot.tX + s_turret.getTurretAngle()),
      new RunCommand(() -> s_drive.tankDriveVolts(0,0), s_drive)
    );
  }
}

