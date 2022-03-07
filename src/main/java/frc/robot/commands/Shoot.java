// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class Shoot extends SequentialCommandGroup {
  /** Creates a new Shoot. */
  public Shoot(Index index, Shooter shooter, Hood hood, Turret turret, double shooterSetpoint, double hoodSetpoint, double turretSetpoint) {
    // addCommands(
    //   new ConditionalCommand( // CC1
    //     new RunCommand(() -> index.setIndexManual(0.3), index),  // T1
    //     new ConditionalCommand( // CC2 F1
    //       new ConditionalCommand( // CC3 T2
    //         parallel(new RunCommand(() -> turret.setTurret(turretSetpoint), turret), 
    //                 new RunCommand(() -> hood.setHood(hoodSetpoint), hood), 
    //                 new RunCommand(() -> shooter.setShooter(shooterSetpoint), shooter)), // T3
    //         parallel(new RunCommand(() -> turret.sweepTurret(true), turret),
    //                 new RunCommand(() -> shooter.setShooter(1000), shooter)),  // F3
    //         () -> Robot.hasTarget()), // B3
    //       parallel(new RunCommand(() -> turret.setTurret(0), turret), 
    //               new RunCommand(() -> hood.setHood(100), hood), 
    //               new RunCommand(() -> shooter.setShooter(1000), shooter)), // F2
    //       () -> index.hasCargo()), // B2
    //     () -> shooter.shooterIsFinished() && turret.turretIsFinished()) // B1
    // );

    addCommands(
      new ConditionalCommand( // C1
        new ConditionalCommand( // T1 C2
          new ConditionalCommand( // T2 C3
            new RunCommand(() -> index.setIndexManual(0.7), index), // T3
            parallel(new RunCommand(() -> turret.setTurret(turretSetpoint), turret), 
                    new RunCommand(() -> hood.setHood(hoodSetpoint), hood), 
                    new RunCommand(() -> shooter.setShooter(shooterSetpoint), shooter)), // F3
            () -> shooter.shooterIsFinished() && turret.turretIsFinished()), // B3
          parallel(new RunCommand(() -> turret.sweepTurret(true), turret),
                  new RunCommand(() -> shooter.setShooter(1000), shooter),
                  new RunCommand(() -> hood.setHood(10), hood)), // F2
          () -> Robot.hasTarget()), // B2
        new PrintCommand("No Cargo Available"), // F1
        () -> index.hasCargo()) // B1
    );
  }
}
