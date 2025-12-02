// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package binarycrows.robot;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import binarycrows.robot.CrowMotion.UserSide.CMConfig;
import binarycrows.robot.SeasonCode.Constants.CrowMotionConstants;
import binarycrows.robot.SeasonCode.Constants.FieldConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveSubStateManager;
import binarycrows.robot.Utils.LogIOInputs;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import gg.questnav.questnav.QuestNav;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {

  @SuppressWarnings("rawtypes")
  private ArrayList<SubStateManager> subStateManagers;

  
  QuestNav questNav = new QuestNav();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    
    MainStateManager.getInstance(); // Initialize the MainStateManager

    
  }
  @SuppressWarnings("resource")
  @Override
  public void robotInit() {
      // Set up data receivers & replay source
      if (RobotBase.isReal()) {
          // Running on a real robot, log to a USB stick ("/U/logs")
          Logger.addDataReceiver(new WPILOGWriter());
          Logger.addDataReceiver(new NT4Publisher());
          new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
      } else {
        if (RobotBase.isSimulation()) {
          // Running a physics simulator, log to NT
          Logger.addDataReceiver(new NT4Publisher());
        } else {
          // Replaying a log, set up replay source
          //setUseTiming(true); // Run as fast as possible
          String logPath = LogFileUtil.findReplayLog();
          Logger.setReplaySource(new WPILOGReader(logPath));
          Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }
      }
      Logger.start();

      // Place your robot's substate managers in this call!
      MainStateManager.getInstance().registerSubStateManagers(
        new DriveSubStateManager()
      );


      StateTable.putValue("SlowMode", false);

      StateTable.putValue("AxisLock", false);
      
      StateTable.putValue("IsDriverControlled", true);

      subStateManagers = MainStateManager.getInstance().getSubStateManagers();

      Keybinds.createKeybinds();

      DriveSubStateManager driveSubStateManager = DriveSubStateManager.getInstance();
      CMConfig.init(
        driveSubStateManager::getRobotPoseCM,
        driveSubStateManager::getRobotVelocityCM,
        driveSubStateManager::driveCM,
        SwerveDriveConstants.distanceBetweenCentersOfRightAndLeftWheels,
        SwerveDriveConstants.distanceBetweenCentersOfFrontAndBackWheels,
        MetaConstants.isBlueAlliance,
        ()->false,
        FieldConstants.fieldWidthMeters,
        FieldConstants.fieldLengthMeters,
        1.5,
        10,
        4.3,
        3.5,
        4,
        0,
        .08,
        .05,
        4.311,
        240,
        480,
        480,
        1,
        5,
        .5,
        1.5,
        10
        );
  }

  @Override
  public void robotPeriodic() {
    questNav.commandPeriodic();
    subStateManagers.forEach(subStateManager -> {
        subStateManager.periodic();
        LogIOInputs.logToStateTable(subStateManager.activeStateRequest, subStateManager.toString() + "/ActiveStateRequest");

        LogIOInputs.logToStateTable(subStateManager.activeStateRequest.getStateRequestType().name(), subStateManager.toString() + "/ActiveStateRequest/Name");
    });
  }

  @Override
  public void autonomousInit() {
    System.out.println("Autonomous init");
    StateTable.putValue("IsDriverControlled", false);
    MetaConstants.startedAutonomous = false;
  }

  @Override
  public void autonomousPeriodic() {
    if (!MetaConstants.startedAutonomous) {

      AutonManager.runAuton();
      MetaConstants.startedAutonomous = true;
    }
  }

  @Override
  public void teleopInit() {
    StateTable.putValue("IsDriverControlled", true);
  }

  @Override
  public void teleopPeriodic() {
    Keybinds.periodic();

  }

  @Override
  public void disabledInit() {
    StateTable.putValue("IsDriverControlled", false);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  
}
