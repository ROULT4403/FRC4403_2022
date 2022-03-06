package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class AutoBackUp0 extends CommandBase{

	// Instantiate subsystems
	public Shooter s_shooter;
	public Hood s_hood;
	public Turret s_turret;
	public Index s_index;
  public Drivetrain s_drivetrain;
  public boolean shooterIsFinished;
  public boolean hasShot;
	
	public AutoBackUp0(Drivetrain drivetrain, Shooter shooter, Hood hood, Turret turret, Index index) {
		this.s_shooter = shooter;
		this.s_hood = hood;
		this.s_turret = turret;
		this.s_index = index;
    this.s_drivetrain = drivetrain;

		// Declare subsystem dependencies
		addRequirements(s_shooter, s_turret, s_hood, s_index);
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void initialize() {
    shooterIsFinished = false;
    hasShot = false;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
    if (!hasShot) {
      shooterIsFinished = s_shooter.shooterIsFinished();

      // All setpoints not final
      s_hood.setHood(5);
      s_shooter.setShooter(2000);

      if (shooterIsFinished) {
        s_index.setIndexManual(0.3);
        s_drivetrain.resetEncoders();
        hasShot = true;
      }
    }

    s_drivetrain.driveDistance(-2.5);

	}	

	// Called once the command ends or is interrupted.
	@Override
  public void end(boolean interrupted) {
	}
}