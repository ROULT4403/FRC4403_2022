// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBackUpDelta extends SequentialCommandGroup {
  /** Creates a new AutoBackUp01. */
  public AutoBackUpDelta(Drivetrain s_drivetrain, Index s_index, Shooter s_shooter, Hood s_hood, Turret s_turret, Intake s_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(s_intake::toggleIntakeRelease, s_intake),
      new RunCommand(() -> s_drivetrain.driveDistance(AutoConstants.autoDistance), s_drivetrain)
      .alongWith(new RunCommand(() -> s_intake.setIntake(0.3), s_intake))
      .alongWith(new RunCommand(() -> s_index.setIndex(0.3), s_index).withTimeout(3))
      .until(() -> s_drivetrain.driveDistanceIsFinished()),
      new RunCommand(() -> s_drivetrain.turnToAngle(AutoConstants.autoTwoAngle), s_drivetrain).until(() -> s_drivetrain.turnToAngleIsFinished()),
      parallel(
        new RunCommand(() -> s_shooter.setShooter(s_shooter.getShooterTargetSpeed()), s_shooter),
        new RunCommand(() -> s_hood.setHood(10), s_hood),
        new RunCommand(() -> s_turret.setTurret(-70), s_turret),
        new WaitCommand(3.5).andThen(new RunCommand(() -> s_index.setIndexManual(0.3), s_index))
      ).withTimeout(7.5),
      new RunCommand(() -> s_drivetrain.driveDistance(AutoConstants.autoTwoDistance), s_drivetrain)
      .alongWith(new RunCommand(() -> s_intake.setIntake(0.3), s_intake))
      .alongWith(new RunCommand(() -> s_index.setIndex(0.3), s_index).withTimeout(3))
      .until(() -> s_drivetrain.driveDistanceIsFinished()),
      parallel(
        new RunCommand(() -> s_shooter.setShooter(s_shooter.getShooterTargetSpeed()), s_shooter),
        new RunCommand(() -> s_hood.setHood(s_hood.getHoodTargetAngle()), s_hood),
        new RunCommand(() -> s_turret.setTurret(-70), s_turret),
        new WaitCommand(3.5).andThen(new RunCommand(() -> s_index.setIndexManual(0.3), s_index))
      )
    );
  }
}














