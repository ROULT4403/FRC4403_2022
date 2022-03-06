// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.ShootBasic;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBackUp00 extends SequentialCommandGroup {
  /** Creates a new AutoBackUp00. */
  public AutoBackUp00(Drivetrain s_drivetrain, Index s_index, Shooter s_shooter, Hood s_hood, Turret s_turret) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShootBasic(s_index, s_shooter, s_hood, s_turret, s_shooter.getShooterTargetSpeed(), s_hood.getHoodTargetAngle(), Robot.tX + s_turret.getTurretAngle()).,
      new RunCommand(() -> s_drivetrain.driveDistance(2.5), s_drivetrain)
    );
  }
}
