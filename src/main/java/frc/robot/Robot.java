// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  public static Trajectory path;
  public static Trajectory path2;
  public static PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);

  // Instantiate NetworkTables
  NetworkTable table;
  NetworkTableInstance nInst;

  NetworkTableEntry xEntry;
  NetworkTableEntry dEntry;
  NetworkTableEntry vEntry;

  // Declare vision variables
  public static boolean tV;
  public static double tX;
  public static double tD;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   * @param
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture(1);

    // Creates trajectories to be used in autonomous
    String trajectoryJSON = "pathplanner/generatedJSON/PathPlannerTest1.wpilib.json";
    String trajectoryJSON2 = "pathplanner/generatedJSON/PathPlannerTest2.wpilib.json";
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);

    // Uses try to catch IOException caused by inexistent file or wrong path
    try {
      path = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      path2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
    
    } catch (IOException e) {
      System.out.println("Error loading path");
      System.out.println(e);
      SmartDashboard.putString("PathStatus", "Error Loading Path");
    }
    
    // Initialize table
    nInst = NetworkTableInstance.getDefault();
    table = nInst.getTable("tablaCool");

    // Initialize entries
    xEntry = table.getEntry("tX");
    dEntry = table.getEntry("tD");
    vEntry = table.getEntry("tV");
    
    new Thread(() -> {
      UsbCamera camera = CameraServer.startAutomaticCapture(0);
      camera.setResolution(320, 240);
      
      // CvSink cvsink = CameraServer.getVideo();
      // CvSource outputstream = CameraServer.putVideo("Test", 640, 480);

      // Mat source = new Mat();
      // Mat output = new Mat();
      // while (!Thread.interrupted()){
      //   cvsink.grabFrame(source);
      //   Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
      //   outputstream.putFrame(output);
      // }
    }).start();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  
    CommandScheduler.getInstance().run();

    //Shuffleboard.getTab("Pre-Match").add("Auto Mode",).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 1).withPosition(0, 0); // place it in the top-left corner
    
    
    Shuffleboard.getTab("Pre-Match").add("Voltage",pdp.getVoltage())
    .withWidget(BuiltInWidgets.kVoltageView).withSize(2,1).withPosition(2,0);
    Shuffleboard.getTab("Pre-Match").add("CurrentIntake",pdp.getCurrent(9))
    .withWidget(BuiltInWidgets.kTextView).withSize(1,1).withPosition(4,0);
    Shuffleboard.getTab("Pre-Match").add("FalconIntake temp",pdp.getTemperature())
    .withWidget(BuiltInWidgets.kDial).withSize(1,1).withPosition(5,0);
    Shuffleboard.getTab("Pre-Match").add("FalconShooter temp",pdp.getTemperature())
    .withWidget(BuiltInWidgets.kDial).withSize(1,1).withPosition(6,0);
    Shuffleboard.getTab("Pre-Match").add("FalconIndex temp",pdp.getTemperature())
    .withWidget(BuiltInWidgets.kDial).withSize(1,1).withPosition(7,0);

    // SmartDashboard.putNumber("CurrenttopLeft", pdp.getCurrent(15));
    // SmartDashboard.putNumber("CurrenttopRight", pdp.getCurrent(14));
    // SmartDashboard.putNumber("CurrentbottomLeft", pdp.getCurrent(0));
    // SmartDashboard.putNumber("CurrentbottomRight", pdp.getCurrent(1));

    // Update vision variables
    tX = xEntry.getDouble(0);    
    tD = dEntry.getDouble(0); 
    tV = vEntry.getBoolean(false);

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {

    Shuffleboard.getTab("Post-Match").add("Voltage",pdp.getVoltage())
    .withWidget(BuiltInWidgets.kVoltageView).withSize(2,1).withPosition(0,0);
    Shuffleboard.getTab("Post-Match").add("CurrentIntake",pdp.getCurrent(9))
    .withWidget(BuiltInWidgets.kTextView).withSize(1,1).withPosition(2,0);
    Shuffleboard.getTab("Post-Match").add("FalconIntake temp",pdp.getTemperature())
    .withWidget(BuiltInWidgets.kDial).withSize(1,1).withPosition(3,0);
    Shuffleboard.getTab("Post-Match").add("FalconShooter",pdp.getTemperature())
    .withWidget(BuiltInWidgets.kDial).withSize(1,1).withPosition(4,0);
    Shuffleboard.getTab("Post-Match").add("FalconIndex temp",pdp.getTemperature())
    .withWidget(BuiltInWidgets.kDial).withSize(1,1).withPosition(5,0);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

