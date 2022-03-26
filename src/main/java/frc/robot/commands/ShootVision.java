// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class ShootVision extends CommandBase {
  /** Creates a new ShootBasic. */
  public Index s_index;
  public Shooter s_shooter;
  public Hood s_hood;
  public Turret s_turret;
  public boolean shooterIsFinished;
  public double counterVariable;

  public ShootVision(Index index, Shooter shooter, Hood hood, Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_index = index; 
    this.s_shooter = shooter; 
    this.s_hood = hood;
    this.s_turret = turret;    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterIsFinished = false;
    counterVariable = -5;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (counterVariable > 20) {
    //   counterVariable = -5;
    // } 
    
    s_shooter.setShooter(s_shooter.getShooterTargetSpeed());
    s_hood.setHood(s_hood.getHoodTargetAngle());
    s_turret.setTurret(Robot.tX + s_turret.getTurretAngle());
    
    // SmartDashboard.putBoolean("SBshooterIsFinished", shooterIsFinished);
    SmartDashboard.putBoolean("shooterCounter", counterVariable > 0);
    if (shooterIsFinished/* || counterVariable > 0*/) {
      s_index.setIndexManual(0.5);
      // ++counterVariable;
    }
    shooterIsFinished = s_shooter.getShooterIsFinished();
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterIsFinished = false;
    counterVariable = -5;
    SmartDashboard.putBoolean("shooterCounter", counterVariable > 0);
    SmartDashboard.putBoolean("SBshooterIsFinished", shooterIsFinished);
    SmartDashboard.putNumber("ShooterTargetSpeed", s_shooter.getShooterTargetSpeed());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
