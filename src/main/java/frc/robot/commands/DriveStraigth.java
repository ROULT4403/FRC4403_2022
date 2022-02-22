package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveStraigth extends PIDCommand{

  public DriveStraigth(Drivetrain s_drive, double speed) {
    super(new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI,
			DrivetrainConstants.kD),s_drive::getYaw, s_drive::getYaw,
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
