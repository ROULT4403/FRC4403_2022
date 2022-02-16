// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.*;
import frc.robot.commands.Auto.Auto1;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain s_drive = new Drivetrain();
  private final Intake s_intake = new Intake();
  private final Index s_index = new Index();
  private final Shooter s_shooter = new Shooter();
  private final Hood s_hood = new Hood();
  private final Turret s_turret = new Turret();

  // Drivetrain Joystick
  Joystick driver = new Joystick(0);
  // Mechanism Joystick
  Joystick controller = new Joystick(1);
  
  // Drivetrain Joystick Buttons
  Button d_A = new JoystickButton(driver, 1);
  Button d_B = new JoystickButton(driver, 2);
  Button d_X = new JoystickButton(driver, 3);
  Button d_Y = new JoystickButton(driver, 4);
  Button d_LB = new JoystickButton(driver, 5);
  Button d_RB= new JoystickButton(driver, 6);
  Button d_Select = new JoystickButton(driver, 7);
  Button d_Start = new JoystickButton(driver, 8);
  Button d_LSClick = new JoystickButton(driver, 9);
  Button d_RSClick = new JoystickButton(driver, 10);
  POVButton d_Pad0 = new POVButton(driver, 0);
  POVButton d_Pad90 = new POVButton(driver, 90);
  POVButton d_Pad180 = new POVButton(driver, 180);
  POVButton d_Pad270 = new POVButton(driver, 270);

  // Mechanism Joystick Buttons
  Button c_A = new JoystickButton(controller, 1);
  Button c_B = new JoystickButton(controller, 2);
  Button c_X = new JoystickButton(controller, 3);
  Button c_Y = new JoystickButton(controller, 4);
  Button c_LB = new JoystickButton(controller, 5);
  Button c_RB= new JoystickButton(controller, 6);
  Button c_Select = new JoystickButton(controller, 7);
  Button c_Start = new JoystickButton(controller, 8);
  Button c_LSClick = new JoystickButton(controller, 9);
  Button c_RSClick = new JoystickButton(controller, 10);
  POVButton c_Pad0 = new POVButton(controller, 0);
  POVButton c_Pad90 = new POVButton(controller, 90);
  POVButton c_Pad180 = new POVButton(controller, 180);
  POVButton c_Pad270 = new POVButton(controller, 270);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // Default Drive Command
    s_drive.setDefaultCommand(new RunCommand(() -> s_drive.drive(-driver.getRawAxis(1), driver.getRawAxis(4)), s_drive));
    // Intake Default Command
    s_intake.setDefaultCommand(new RunCommand(() -> s_intake.setIntake(0), s_intake));
    // Index Default Command
    s_index.setDefaultCommand(new RunCommand(() -> s_index.setIndexManual(0), s_index));
    // Shooter Default Command
    s_shooter.setDefaultCommand(new RunCommand(() -> s_shooter.setShooterManual(0), s_shooter));
    // Hood Default Command
    s_hood.setDefaultCommand(new RunCommand(() -> s_hood.setHoodManual(0), s_hood));
    // Turret Default Command
    s_turret.setDefaultCommand(new RunCommand(() -> s_turret.setTurretManual(0), s_turret));
  }

  /**
   * Use this method to define your button->command mappings. 
   */
  private void configureButtonBindings() {
    // Dual Controller
    if(DriverStation.isJoystickConnected(1)){
      // Driver Controls
      d_RSClick.whenPressed(new InstantCommand(s_drive::toggleDogShift, s_drive));
      // Intake Release
      d_LB.whenPressed(new InstantCommand(s_intake::toggleIntakeRelease, s_intake));
      // Algorithm intake
      d_RB.whileHeld(new RunCommand(() -> s_intake.setIntake(0.3, true), s_intake).alongWith(new RunCommand(() -> s_index.setIndex(0.25, s_intake.detectedCargoIntake), s_index)));
      // Algorithm outake
      d_LSClick.whileHeld(new RunCommand(() -> s_intake.setIntake(-0.4), s_intake).alongWith(new RunCommand(() -> s_index.setIndexManual(-0.25), s_index)));

      // Controller Controls
      c_RB.whenHeld(new ShootBasic(s_index, s_shooter));
      return;
    }

    // Single Controller
    // Driver Controls
    d_RSClick.whenPressed(new InstantCommand(s_drive::toggleDogShift, s_drive));

    // Shooter Commands
    // Manual Hood
    d_Pad0.whileHeld(new RunCommand(() -> s_hood.setHoodManual(HoodConstants.hoodOutput), s_shooter));
    d_Pad180.whileHeld(new RunCommand(() -> s_hood.setHoodManual(-HoodConstants.hoodOutput), s_shooter));
    // Manual Turret
    d_Start.whenPressed(new RunCommand(() -> s_turret.setTurret(Robot.tx + s_turret.getTurretAngle()), s_shooter));
    d_Pad90.whenPressed(new RunCommand(() -> s_turret.setTurret(-TurretConstants.turretOutput), s_shooter));
    d_Pad270.whileHeld(new RunCommand(() -> s_turret.setTurretManual(TurretConstants.turretOutput), s_shooter));
    // Shooting Algorithm
    d_X.whenHeld(new ShootBasic(s_index, s_shooter));
    
    // Intake & Index
    // Intake Release
    d_LB.whenPressed(new InstantCommand(s_intake::toggleIntakeRelease, s_intake));
    // Algorithm intake
    d_RB.whileHeld(new RunCommand(() -> s_intake.setIntake(0.3, true), s_intake).alongWith(new RunCommand(() -> s_index.setIndex(0.25, s_intake.detectedCargoIntake), s_index)));
    // Algorith Outake
    d_LSClick.whileHeld(new RunCommand(() -> s_intake.setIntake(-0.4, false), s_intake).alongWith(new RunCommand(() -> s_index.setIndexManual(-0.25), s_index)));
    // Manual Intake & Index
    d_B.whileHeld(new RunCommand(() -> s_intake.setIntake(controller.getRawAxis(5))));
    d_B.whileHeld(new RunCommand(() -> s_index.setIndex(controller.getRawAxis(0))));

    d_Select.whenPressed(new InstantCommand(s_turret::resetAngle, s_turret));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      return new Auto1(s_drive);
    }

  /**
   * Get drive command based on controller input
   * @return the command for drivetrain
   */
  public Command getDriveCommand() {
    if (Math.abs(driver.getRawAxis(4)) < 0.05){
      return new DriveStraigth(s_drive, driver.getRawAxis(1));
    } else {
      return new RunCommand(() -> s_drive.drive(driver.getRawAxis(1), driver.getRawAxis(4)), s_drive);
    }
  }
}