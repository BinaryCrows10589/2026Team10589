// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package binarycrows.robot;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import binarycrows.robot.CrowMotion.UserSide.CMConfig;
import binarycrows.robot.SeasonCode.Autons.DepotTrench_Wall_Shoot_L_Shoot_P_Shoot;
import binarycrows.robot.SeasonCode.Constants.FieldConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.PoseEstimatorConstants;
import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.CANdle.CANdleSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Flywheel.FlywheelSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot.PivotSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Rollers.IntakeRollersSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Shooting.ShootingSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Transit.TransitSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretSubStateManager;
import binarycrows.robot.SeasonCode.Utils.Climbing;
import binarycrows.robot.Utils.Auton.Auton;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  public static double timeUntilHubIsActive = -1;

  public static boolean isDriverControlled = true;

  public static boolean isSlowMode;

  public static boolean forceRobotRelative;

  private boolean shift1Active = false;

  private Field2d dashboardField;
  private Supplier<Pose2d> robotPoseSupplier;
  
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
        //new ClimberSubStateManager(),
        new CANdleSubStateManager(),
        new ShootingSubStateManager()
      );

    
      MainStateManager.getInstance().start();


      isSlowMode= false;
      
      isDriverControlled = true;

      forceRobotRelative = false;
      

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
        2,
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




    // Initialize autonomous chooser
    Auton defaultAuton = new Auton(DepotTrench_Wall_Shoot_L_Shoot_P_Shoot.startingPoint, DepotTrench_Wall_Shoot_L_Shoot_P_Shoot::getAutonomous);
    chooser.addDefaultOption("Depot Trench Wall: Shoot, L, Shoot, P, Shoot", defaultAuton);
    
    onAutonSelect(defaultAuton); // Initialize first autonomous that is selected

    chooser.onChange(this::onAutonSelect);


    // Final updates
    updateAlliance();
    DriveSubStateManager.getInstance().resetRobotPose();

    dashboardField = new Field2d();
    robotPoseSupplier = DriveSubStateManager.getInstance()::getRobotPose;

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

        Logger.recordOutput(subStateManager.toString() + "/ActiveStateRequest", subStateManager.activeStateRequest.getAsLoggable());
        Logger.recordOutput(subStateManager.toString() + "/ActiveStateRequest/Name", subStateManager.activeStateRequest.getStateRequestType().name());
    });

    long currentTime = System.currentTimeMillis();
    if (frameStartTime != -1) {
        double frameTime = (currentTime - frameStartTime) / 1000.0;
        
        averageFrameTime = (averageFrameTime * .9) + (frameTime * .1);
    }
    frameStartTime = currentTime;

    timeUntilHubIsActive = secondsUntilHubIsActive();

    Logger.recordOutput("SecondsUntilHubIsActive", timeUntilHubIsActive);
    Logger.recordOutput("ActiveInShift1", shift1Active);

    dashboardField.setRobotPose(robotPoseSupplier.get());
    SmartDashboard.putData("Field", dashboardField);

  }

  public void onAutonSelect(Auton auton) {
    if (auton != null) {
      auton.buildAuton();
    }
  }

  @Override
  public void autonomousInit() {
    isDriverControlled = false;
    MetaConstants.startedAutonomous = false;
  }

  @Override
  public void autonomousPeriodic() {
    if (!MetaConstants.startedAutonomous) {
      Auton auton = chooser.get();
      if (auton.builtAuton == null) {
        System.out.println("NOT BUILT!");
        auton.buildAuton(); // In case it wasn't built for some reason (should NEVER be necessary!)
      }
      DriveSubStateManager.getInstance().setRobotPose(auton.startingPoint);
      DriveSubStateManager.getInstance().setSimSwerveTurnRotations(auton.startingPoint.getRotation());

      auton.builtAuton.dispatchSelf();
      MetaConstants.startedAutonomous = true;
    }
  }

  @Override
  public void teleopInit() {
    isDriverControlled = true;
    //Climbing.climbRight(); // Sim testing
  }

  @Override
  public void teleopPeriodic() {
    Keybinds.periodic();

  }

  @Override
  public void disabledInit() {
    isDriverControlled = false;
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
  // Modified from WPILib docs:
  // https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
  // Returns seconds until the hub is active to allow for preshooting.
  // Returns - if the hub is inactive, magnitude is time till active
  // Returns 0 if the hub is active and/or time is incalcuable
  public double secondsUntilHubIsActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return 0;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return 0;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return 0;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as it's likely early in teleop.
    if (gameData.isEmpty()) {
      return 0;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return 0;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      if (shift1Active) return 105 - matchTime;
      else return 130 - matchTime;
    } else if (matchTime > 105) {
      // Shift 1
      if (shift1Active) return 105 - matchTime;
      else return matchTime - 105;
    } else if (matchTime > 80) {
      // Shift 2
      if (!shift1Active) return 80 - matchTime;
      else return matchTime - 80;
    } else if (matchTime > 55) {
      // Shift 3
      if (shift1Active) return 55 - matchTime;
      else return matchTime - 55;
    } else if (matchTime > 30) {
      // Shift 4
      if (!shift1Active) return 30 - matchTime;
      else return matchTime - 30;
    } else {
      // End game, hub always active.
      return 0;
  }
}

  
}
