package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class ShootSubAuto extends CommandBase{

	// Instantiate subsystems
	public Shooter s_shooter;
	public Hood s_hood;
	public Turret s_turret;
	public Index s_index;
	
	public ShootSubAuto(Shooter shooter, Hood hood, Turret turret, Index index) {
		this.s_shooter = shooter;
		this.s_hood = hood;
		this.s_turret = turret;
		this.s_index = index;

		// Declare subsystem dependencies
		addRequirements(s_shooter, s_turret, s_hood, s_index);
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		// If balls in robot
		if (s_index.hasCargo()) {
			// If target in range
			if (Robot.tv) {
				// Start turret and hood
				new ParallelCommandGroup(new RunCommand(() -> s_turret.setTurret(Robot.tx + s_turret.getTurretAngle()), s_shooter), 
                                new RunCommand(() -> s_hood.setHood(s_hood.getHoodAngle()), s_shooter), 
                                new RunCommand(() -> s_shooter.setShooter(s_shooter.getShooterSpeed()), s_shooter));

				if (s_shooter.shooterIsFinished() && s_turret.turretIsFinished()) {
					new RunCommand(() -> s_index.setIndexManual(0.7), s_index);
				}

			} else {
				// Move turret until target on sight
				new RunCommand(() -> s_turret.sweepTurret(true), s_shooter);
			}
		}
	}	

	// Called once the command ends or is interrupted.
	@Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("VisionStatus", "Shot");
  }
}
