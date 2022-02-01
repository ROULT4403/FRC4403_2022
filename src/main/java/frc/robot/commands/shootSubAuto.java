package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class shootSubAuto extends CommandBase{

	// Instantiate subsystems
	public Shooter s_shooter;
	public Index s_index;


	public shootSubAuto(Shooter shooter, Index index) {
		this.s_shooter = shooter;
		this.s_index = index;

		// Declare subsystem dependencies
		addRequirements(s_shooter, s_index);
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
			// Start flywheel  
			new RunCommand(() -> s_shooter.shoot(100), s_shooter);

			// If target in range
			if (Robot.tv) {
				// Start turret and hood
				new ParallelCommandGroup(new RunCommand(() -> s_shooter.setTurret(Robot.tx), s_shooter), new RunCommand(() -> s_shooter.setHood(s_shooter.getHoodAngle()), s_shooter), new RunCommand(() -> s_shooter.shoot(s_shooter.getShootSpeed()), s_shooter));

				if (s_shooter.shootIsFinished() && s_shooter.turretIsFinished()) {
					new RunCommand(() -> s_index.indexControl(0.5), s_index);
				}

			} else {
				// Move turret until target on sight
				new RunCommand(() -> s_shooter.sweepTurret(true), s_shooter);
			}
		}
	}	

	// Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("VisionStatus", "Shot");
  }


}
