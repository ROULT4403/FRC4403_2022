// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBackUpBeta extends SequentialCommandGroup {
  /** Creates a new AutoBackUp00. */
  public AutoBackUpBeta(Drivetrain s_drivetrain, Index s_index, Shooter s_shooter, Hood s_hood, Turret s_turret, Intake s_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(s_intake::toggleIntakeRelease, s_intake),
      new InstantCommand(s_intake::toggleIntakeRelease, s_intake),
      new RunCommand(() -> s_drivetrain.drive(0.7, 0), s_drivetrain).withTimeout(0.75)
      .alongWith(new RunCommand(() -> s_intake.setIntake(0.5), s_intake))
      .alongWith(new RunCommand(() -> s_index.setIndex(0.5), s_index)).withTimeout(3),
      new RunCommand(() -> s_drivetrain.drive(0, -0.7), s_drivetrain).withTimeout(1.8),
      parallel(
        new RunCommand(() -> s_shooter.setShooter(s_shooter.getShooterTargetSpeed()), s_shooter),
        new RunCommand(() -> s_hood.setHood(s_hood.getHoodTargetAngle()), s_hood),
        new RunCommand(() -> s_turret.setTurret(s_turret.getTurretAngle() + Robot.tX), s_turret),
        new WaitCommand(3).andThen(new RunCommand(() -> s_index.setIndexManual(0.5), s_index))
      )
      .alongWith(new RunCommand(()-> s_drivetrain.drive(0, 0), s_drivetrain))
    );

  }
}
