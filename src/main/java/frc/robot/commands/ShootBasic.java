// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ShootBasic extends CommandBase {
  /** Creates a new ShootBasic. */
  public Index s_index;
  public Shooter s_shooter;
  public Hood s_hood;
  public Turret s_turret;
  public boolean shooterIsFinished;
  public boolean isballShot;

  public double s_shooterSetpoint;
  public double s_hoodSetpoint;
  public double s_turretSetpoint;

  public ShootBasic(Index index, Shooter shooter, Hood hood, Turret turret, double shooterSetpoint, double hoodSetpoint, double turretSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_index = index; 
    this.s_shooter = shooter; 
    this.s_hood = hood;
    this.s_turret = turret;

    this.s_shooterSetpoint = shooterSetpoint;
    this.s_hoodSetpoint = hoodSetpoint;
    this.s_turretSetpoint = turretSetpoint;      
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterIsFinished = false;
    isballShot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterIsFinished = s_shooter.shooterIsFinished();
    isballShot = false;

    s_shooter.setShooter(s_shooterSetpoint);
    s_hood.setHood(s_hoodSetpoint);
    s_turret.setTurret(s_turretSetpoint);

    SmartDashboard.putBoolean("SBshooterIsFinished", shooterIsFinished);
    if (shooterIsFinished) {
      s_index.setIndexManual(0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
