package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class AutoBackUp1 extends CommandBase{

	// Instantiate subsystems
  public Drivetrain s_drivetrain;
	public Shooter s_shooter;
	public Hood s_hood;
	public Turret s_turret;
	public Index s_index;
	public Intake s_intake;

  public boolean shooterIsFinished;
  public boolean hasShot;
	public boolean grabbedCargo;
	public boolean hasDriven;
	public boolean hasTurned;

	public AutoBackUp1(Drivetrain drivetrain, Shooter shooter, Hood hood, Turret turret, Index index, Intake intake) {
    this.s_drivetrain = drivetrain;
		this.s_shooter = shooter;
		this.s_hood = hood;
		this.s_turret = turret;
		this.s_index = index;
		this.s_intake = intake;

		// Declare subsystem dependencies
		addRequirements(s_shooter, s_turret, s_hood, s_index);
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void initialize() {
		shooterIsFinished = false;
		hasShot = false;
		grabbedCargo = false;
		hasDriven = false;
		hasTurned = false;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// All setpoints not final
		// If distance has been driven
		if (hasDriven) {
			// If cargo has been grabbed
			if (grabbedCargo) {
				// Sets cargo to true
				grabbedCargo = true;
				// If robot has turned
				if (hasTurned) {
					if (!hasShot) {
						shooterIsFinished = s_shooter.shooterIsFinished();
						
						s_turret.setTurret(90);
						s_hood.setHood(5);
						s_shooter.setShooter(2000);
						
						if (shooterIsFinished) {
							s_index.setIndexManual(0.3);
							hasShot = true;
						}
					}
				}
			} else {
				// Picks up balls
				grabbedCargo = s_index.isCargoAvailable();
				s_intake.setIntake(0.3);
			}
		} else {
			s_drivetrain.driveDistance(2.5);
		}
		

	}	

	// Called once the command ends or is interrupted.
	@Override
  public void end(boolean interrupted) {
	}
}