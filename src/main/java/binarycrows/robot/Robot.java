// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package binarycrows.robot;

import java.util.ArrayList;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import binarycrows.robot.SeasonCode.Subsystems.MotorTest.MotorSubStateManager;
import binarycrows.robot.Utils.Gamepad.GenericGamepad;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {

  @SuppressWarnings("rawtypes")
  private ArrayList<SubStateManager> subStateManagers;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    
    MainStateManager.getInstance(); // Initialize the MainStateManager

    // Place your robot's substate managers in this call!
    MainStateManager.getInstance().registerSubStateManagers(
      new MotorSubStateManager()
    );
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

      StateTable.putValue("isSim", !RobotBase.isReal());

      subStateManagers = MainStateManager.getInstance().getSubStateManagers();

      MainStateManager.getInstance().start();
  }

  @Override
  public void robotPeriodic() {
    subStateManagers.forEach(subStateManager -> {
        subStateManager.periodic();
    });
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

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
