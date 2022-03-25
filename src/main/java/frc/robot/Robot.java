// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.cameraserver.CameraServer;
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
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  //PDP 
  public static PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  
  //AUTO
  //Instantiate trayectories
  //Test trajectories
  public static Trajectory path;
  public static Trajectory path2;
  //Start trajectory
  public static Trajectory trajectoryStart;
  //Auto 1.1 trayectories
  public static Trajectory trajectoryAuto1_1;
  public static Trajectory trajectoryAuto1_1_1;
  public static Trajectory trajectoryAuto1_1_2;
  //Auto 1.2 trajectories
  public static Trajectory trajectoryAuto1_2;
  public static Trajectory trajectoryAuto1_2_1;
  public static Trajectory trajectoryAuto1_2_2Option1;
  public static Trajectory trajectoryAuto1_2_2Option2;
  public static Trajectory trajectoryAuto1_2_2_1Option2;
  //Auto 2.1 trajectories
  public static Trajectory trajectoryAuto2_1;
  public static Trajectory trajectoryAuto2_1_1;

  //Path generation boolean
  boolean pathGenerated = false;

  //VISION
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
    // CameraServer.startAutomaticCapture(1);

    //Start JSON
    String fileStartJSON = "pathplanner/generatedJSON/AutoStart.wpilib.json";
    //Auto1.1 file JSON variables
    String fileAuto1_1JSON = "pathplanner/generatedJSON/Auto1.1.wpilib.json";
    String fileAuto1_1_1JSON = "pathplanner/generatedJSON/Auto1.1.1.wpilib.json";
    String fileAuto1_1_2JSON = "pathplanner/generatedJSON/Auto1.1.2.wpilib.json";
    //Auto1.2 file JSON varibales
    String fileAuto1_2JSON = "pathplanner/generatedJSON/Auto1.2.wpilib.json";
    String fileAuto1_2_1JSON = "pathplanner/generatedJSON/Auto1.2.1.wpilib.json";
    String fileAuto1_2_2Option1JSON = "pathplanner/generatedJSON/Auto1.2.2(option #1).wpilib.json";
    String fileAuto1_2_2Option2JSON = "pathplanner/generatedJSON/Auto1.2.2(option #2).wpilib.json";
    String fileAuto1_2_2_1Option2JSON = "pathplanner/generatedJSON/Auto1.2.2.1(option #2).wpilib.json";
    //Auto2.1 file JSON variables
    String fileAuto2_1JSON = "pathplanner/generatedJSON/Auto2.1.wpilib.json";
    String fileAuto2_1_1JSON = "pathplanner/generatedJSON/Auto2.1.1.wpilib.json";

    /**Creates paths to be used in autonomous*/
    //Auto start path
    Path pathStart = Filesystem.getDeployDirectory().toPath().resolve(fileStartJSON);
    //Auto1.1 path variables
    Path pathAuto1_1 = Filesystem.getDeployDirectory().toPath().resolve(fileAuto1_1JSON);
    Path pathAuto1_1_1 = Filesystem.getDeployDirectory().toPath().resolve(fileAuto1_1_1JSON);
    Path pathAuto1_1_2 = Filesystem.getDeployDirectory().toPath().resolve(fileAuto1_1_2JSON);
    //Auto1.2 path variables
    Path pathAuto1_2 = Filesystem.getDeployDirectory().toPath().resolve(fileAuto1_2JSON);
    Path pathAuto1_2_1 = Filesystem.getDeployDirectory().toPath().resolve(fileAuto1_2_1JSON);
    Path pathAuto1_2_2Option1 = Filesystem.getDeployDirectory().toPath().resolve(fileAuto1_2_2Option1JSON);
    Path pathAuto1_2_2Option2 = Filesystem.getDeployDirectory().toPath().resolve(fileAuto1_2_2Option2JSON);
    Path pathAuto1_2_2_1Option2 = Filesystem.getDeployDirectory().toPath().resolve(fileAuto1_2_2_1Option2JSON);
    //Auto 2.1 path variables
    Path pathAuto2_1 = Filesystem.getDeployDirectory().toPath().resolve(fileAuto2_1JSON);
    Path pathAuto2_1_1 = Filesystem.getDeployDirectory().toPath().resolve(fileAuto2_1_1JSON);

    
    // Uses try to catch IOException caused by inexistent file or wrong path
    try {
      trajectoryStart = TrajectoryUtil.fromPathweaverJson(pathStart);
      /**Path generation chooser switch structure*/
      switch (AutoConstants.pathChoose) {
        //Path case 1.1.2 generation
        case "Auto1.1.2":
          trajectoryAuto1_1 = TrajectoryUtil.fromPathweaverJson(pathAuto1_1);
          trajectoryAuto1_1_1 = TrajectoryUtil.fromPathweaverJson(pathAuto1_1_1);
          trajectoryAuto1_1_2 = TrajectoryUtil.fromPathweaverJson(pathAuto1_1_2);
          pathGenerated = true;
          break;
        //Path case 1.2.1 generation
        case "Auto1.2.1":
          trajectoryAuto1_1 = TrajectoryUtil.fromPathweaverJson(pathAuto1_1);
          trajectoryAuto1_2 = TrajectoryUtil.fromPathweaverJson(pathAuto1_2);
          trajectoryAuto1_2_1 = TrajectoryUtil.fromPathweaverJson(pathAuto1_2_1);
          pathGenerated = true;
          break;
        //Path case 1.2.2Option1 generation
        case "Auto1.2.2Option1":
          trajectoryAuto1_1 = TrajectoryUtil.fromPathweaverJson(pathAuto1_1);
          trajectoryAuto1_2 = TrajectoryUtil.fromPathweaverJson(pathAuto1_2);
          trajectoryAuto1_2_2Option1 = TrajectoryUtil.fromPathweaverJson(pathAuto1_2_2Option1);
          pathGenerated = true;
          break;
        //Path case 1.2.2.1Option2 generation
        case "Auto1_2_2_1Option2":
          trajectoryAuto1_1 = TrajectoryUtil.fromPathweaverJson(pathAuto1_1);
          trajectoryAuto1_2 = TrajectoryUtil.fromPathweaverJson(pathAuto1_2);
          trajectoryAuto1_2_2Option2 = TrajectoryUtil.fromPathweaverJson(pathAuto1_2_2Option2);
          trajectoryAuto1_2_2_1Option2 = TrajectoryUtil.fromPathweaverJson(pathAuto1_2_2_1Option2);
          pathGenerated = true;
          break;
        //Path case 2.1.1 generation
        case "Auto2.1.1":
          trajectoryAuto2_1 = TrajectoryUtil.fromPathweaverJson(pathAuto2_1);
          trajectoryAuto2_1_1 = TrajectoryUtil.fromPathweaverJson(pathAuto2_1_1);
          pathGenerated = true;
          break;
        //Default outcome in case of error on pathChoose constant
        default:
          pathGenerated = false;
        }

    }catch (IOException e) {
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
    
    // Update vision variables
    tX = xEntry.getDouble(0);    
    tD = dEntry.getDouble(0); 
    tV = vEntry.getBoolean(false);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

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
    /** This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous to
     * continue until interrupted by another command, remove this line or comment it out.*/ 
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

  public static boolean hasTarget() {
    return tV;
  }
}


