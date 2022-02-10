// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ShootBasic extends CommandBase {
  /** Creates a new ShootBasic. */
  public Index s_index;
  public Shooter s_shooter;

  public ShootBasic(Index index, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_index = index; 
    this.s_shooter = shooter; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_shooter.setShooter(1000);

    if (s_shooter.shooterIsFinished()) {
      s_index.setIndexManual(0.7);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
