// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DefaultDrive extends CommandBase {

  Drivetrain s_drive;
  DoubleSupplier m_forward;
  DoubleSupplier m_rotation;

  public DefaultDrive(Drivetrain subsystem, DoubleSupplier forward, DoubleSupplier rotation) {
    s_drive = subsystem;
    m_forward = forward;
    m_rotation = rotation;
    addRequirements(s_drive);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
  //s_drive.drive(m_forward.getAsDouble(), m_rotation.getAsDouble());
 
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
