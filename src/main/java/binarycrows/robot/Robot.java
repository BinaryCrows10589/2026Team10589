// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package binarycrows.robot;

import java.util.ArrayList;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import binarycrows.robot.CrowMotion.UserSide.CMConfig;
import binarycrows.robot.SeasonCode.Autons.StartOfRamp_ShootPreload_SweepCenterClose_Return;
import binarycrows.robot.SeasonCode.Autons.Test.CMTest2;
import binarycrows.robot.SeasonCode.Constants.FieldConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.PoseEstimatorConstants;
import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.CANdle.CANdleSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Climber.ClimberSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Flywheel.FlywheelSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot.PivotSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Rollers.IntakeRollersSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Transit.TransitSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretSubStateManager;
import binarycrows.robot.SeasonCode.Utils.Shooting;
import binarycrows.robot.Utils.Auton.Auton;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  private static double frameStartTime = -1;

  public static double averageFrameTime = MetaConstants.loopPeriodSeconds;


  private final LoggedDashboardChooser<Auton> chooser = new LoggedDashboardChooser<>("AutonPath");

  
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
        new DriveSubStateManager(),
        new TurretSubStateManager(),
        new TransitSubStateManager(),
        new IntakeRollersSubStateManager(),
        new PivotSubStateManager(),
        new HoodSubStateManager(),
        new FlywheelSubStateManager(),
        new ClimberSubStateManager(),
        new CANdleSubStateManager()
      );

    
      MainStateManager.getInstance().start();


      StateTable.log("SlowMode", false);

      StateTable.log("AxisLock", false);
      
      StateTable.log("IsDriverControlled", true);

      StateTable.log("ForceRobotRelative", false);
      

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
        SwerveDriveConstants.maxSpeedMPS,
        240,
        480,
        480,
        1,
        5,
        .5,
        1.5,
        10
        );

        Shooting.init();



    // Initialize autonomous chooser
    chooser.addDefaultOption("MainAuton", new Auton(StartOfRamp_ShootPreload_SweepCenterClose_Return.startingPoint, StartOfRamp_ShootPreload_SweepCenterClose_Return::getAutonomous));
    chooser.addOption("CMTest2", new Auton(CMTest2.startingPoint, CMTest2::getAutonomous));

    onAutonSelect(chooser.get()); // Initialize first autonomous that is selected

    chooser.onChange(this::onAutonSelect);


    // Final updates
    updateAlliance();
    DriveSubStateManager.getInstance().resetRobotPose();

  }

  


  @SuppressWarnings("unchecked")
  @Override
  public void robotPeriodic() {
    updateAlliance();
    subStateManagers.forEach(subStateManager -> {
        if (subStateManager.activeStateRequest == null) {
          if (subStateManager.defaultState != null) {
            subStateManager.activeStateRequest = subStateManager.defaultState;
          } else {
            return;
          }
        }
        subStateManager.periodic();

        StateTable.log(subStateManager.toString() + "/ActiveStateRequest", subStateManager.activeStateRequest);
        StateTable.log(subStateManager.toString() + "/ActiveStateRequest/Name", subStateManager.activeStateRequest.getStateRequestType().name());
    });

    long currentTime = System.currentTimeMillis();
    if (frameStartTime != -1) {
        double frameTime = (currentTime - frameStartTime) / 1000.0;
        
        averageFrameTime = (averageFrameTime * .9) + (frameTime * .1);
    }
    frameStartTime = currentTime;

    Shooting.periodic();

  }

  public void onAutonSelect(Auton auton) {
    if (auton != null) auton.buildAuton();
  }

  @Override
  public void autonomousInit() {
    StateTable.log("IsDriverControlled", false);
    MetaConstants.startedAutonomous = false;
  }

  @Override
  public void autonomousPeriodic() {
    if (!MetaConstants.startedAutonomous) {
      Auton auton = chooser.get();
      if (auton.getLength() == 0) auton.buildAuton(); // In case it wasn't built for some reason (should NEVER be necessary!)
      DriveSubStateManager.getInstance().setRobotPose(auton.startingPoint);
      auton.dispatchSelf();
      MetaConstants.startedAutonomous = true;
    }
  }

  @Override
  public void teleopInit() {
    StateTable.log("IsDriverControlled", true);
  }

  @Override
  public void teleopPeriodic() {
    Keybinds.periodic();

  }

  @Override
  public void disabledInit() {
    StateTable.log("IsDriverControlled", false);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public void updateAlliance() {
    boolean isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    if (MetaConstants.isBlueAlliance != isBlueAlliance) { // Alliance has switched
        MetaConstants.isBlueAlliance = isBlueAlliance;
        CMConfig.updateAlliance(isBlueAlliance);
        PoseEstimatorConstants.originPosition = isBlueAlliance ? OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide;
        PoseEstimatorConstants.aprilTagLayout.setOrigin(PoseEstimatorConstants.originPosition);
        DriveSubStateManager.getInstance().updatePoseEstimatorAlliance();
    }
  }

  
}
