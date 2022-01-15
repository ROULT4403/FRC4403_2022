// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Defaultdrive extends CommandBase {

  private final Drivetrain s_Drive;
  private final DoubleSupplier v_speed;
  private final DoubleSupplier v_rot;

  public Defaultdrive(Drivetrain subsystem, DoubleSupplier speed, DoubleSupplier rot){
    s_Drive = subsystem;
    v_speed = speed;
    v_rot = rot;
    
    addRequirements(s_Drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Drive.drive(v_speed.getAsDouble(), v_rot.getAsDouble());
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
