// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBackUp0 extends SequentialCommandGroup {
  /** Creates a new AutoBackUp00. */
  public AutoBackUp0(Drivetrain s_drivetrain, Index s_index, Shooter s_shooter, Hood s_hood, Turret s_turret) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(() -> s_drivetrain.drive(-0.5, 0), s_drivetrain).withTimeout(4),
      // new ShootBasic(s_index, s_shooter, s_hood, s_turret, 1900, 10, 0)
      // new ShootVision(s_index, s_shooter, s_hood, s_turret)
      parallel(
        new RunCommand(() -> s_shooter.setShooter(s_shooter.getShooterTargetSpeed()), s_shooter),
        new RunCommand(() -> s_hood.setHood(10), s_hood),
        new RunCommand(() -> s_turret.setTurret(0), s_turret),
        new WaitCommand(5).andThen(new RunCommand(() -> s_index.setIndexManual(0.3), s_index))
      )
      .alongWith(new RunCommand(()-> s_drivetrain.drive(0, 0), s_drivetrain))
      // new ShootBasic(s_index, s_shooter, s_hood, s_turret, 1000, 10, 0)
    );
  }
}
