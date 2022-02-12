package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class straightDrive extends PIDCommand{

  public straightDrive(Drivetrain s_drive, double speed) {
    super(new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI,
			DrivetrainConstants.kD), s_drive::getYaw, 0,
			output -> s_drive.drive(speed, output));

		getController().setTolerance(DrivetrainConstants.kToleranceStraightDriveAngle,
               									 DrivetrainConstants.kToleranceStraightDriveVelocity);

    addRequirements(s_drive);
    }


  @Override
  public boolean isFinished() {
		return getController().atSetpoint();
	}
}
