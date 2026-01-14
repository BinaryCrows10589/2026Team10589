package binarycrows.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import binarycrows.robot.CrowMotion.UserSide.CMAutonPoint;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import binarycrows.robot.SeasonCode.Constants.PoseEstimatorConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveSubStateManager;
import binarycrows.robot.Utils.ConversionUtils;
import binarycrows.robot.Utils.LogIOInputs;
import binarycrows.robot.Utils.Auton.AutonPoint;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class PoseEstimator {
    private PhotonCamera[] photonCameras = {new PhotonCamera("BLModuleCam"), new PhotonCamera("FLModuleCam")};
    private PhotonPoseEstimator[] photonPoseEstimators = new PhotonPoseEstimator[photonCameras.length];
    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private QuestNav questNav = new QuestNav();

    private Notifier visionNotifier = new Notifier(this::addVisionMeasurments);

    private Pose3d lastQuestNavPose;

    // These are for Eli and David's gyro acceleration algorithm
    // It did not increase accuracy so it has been disabled
    /*
    private ChassisSpeeds lastChassisSpeeds;
    private Pose2d lastRobotPose;
    private double dt;
    private long lastFrameStart = -1;
    */

    public PoseEstimator(Rotation2d yawAngle, SwerveModulePosition[] modulePositions) {
        
        configPhotonPoseEstimators();
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
            SwerveDriveConstants.driveKinematics,
            yawAngle,
            modulePositions, 
            new Pose2d(),
            PoseEstimatorConstants.swerveDrivePoseEstimateTrust, PoseEstimatorConstants.visionPoseEstimateTrust);
        PoseEstimatorConstants.aprilTagLayout.setOrigin(PoseEstimatorConstants.originPosition);
        visionNotifier.startPeriodic(.1);

    }

    public void periodic() {

        /*if(this.lastFrameStart != -1) {
            this.dt = (System.currentTimeMillis() - this.lastFrameStart) / 1000.0;
        } else {
            this.dt = 1/50;
        }
        this.lastFrameStart = System.currentTimeMillis();
        */

        try {
            questNav.commandPeriodic();
            this.swerveDrivePoseEstimator.update(DriveSubStateManager.getInstance().gyroOutputs.yawAngle,
                DriveSubStateManager.getInstance().getModulePositions());

            // Accelerometer data algorithm (was less accurate than odometry and didn't help drift)
            /*if(this.lastChassisSpeeds != null && this.lastRobotPose != null && Math.sqrt(this.lastChassisSpeeds.vxMetersPerSecond * this.lastChassisSpeeds.vxMetersPerSecond + this.lastChassisSpeeds.vyMetersPerSecond * this.lastChassisSpeeds.vyMetersPerSecond) > .05) {
                double xPose = (DriveSubStateManager.getInstance().gyroOutputs.xAccelerationMetersPerSecondPerSecond * dt + this.lastChassisSpeeds.vxMetersPerSecond) * dt + this.lastRobotPose.getX();
                double yPose = (DriveSubStateManager.getInstance().gyroOutputs.yAccelerationMetersPerSecondPerSecond * dt + this.lastChassisSpeeds.vyMetersPerSecond) * dt + this.lastRobotPose.getY();
                double timestamp = Timer.getFPGATimestamp();
               swerveDrivePoseEstimator.addVisionMeasurement(new Pose2d(xPose, yPose, this.swerveDrivePoseEstimator.getEstimatedPosition().getRotation()), timestamp, PoseEstimatorConstants.visionPoseEstimateTrust);
            }*/
            LogIOInputs.logToStateTable(this.swerveDrivePoseEstimator.getEstimatedPosition(), "PoseEstimator/EstimatedPosition");

        } catch(Exception E) {
           System.out.println("FAILED TO UPDATE POSE ESTIMATOR: " + E.getMessage());
        }
    }

    public Pose2d getRobotPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void setRobotPose(AutonPoint newRobotPose) {
        setRobotPose(newRobotPose.getAutonPoint());
    }

    public void setRobotPose(Pose2d newRobotPose) {
        DriveSubStateManager.getInstance().resetGyro(newRobotPose.getRotation());
        this.swerveDrivePoseEstimator.resetPosition(DriveSubStateManager.getInstance().getGyroAngleRotation2d(),
        DriveSubStateManager.getInstance().getModulePositions(), newRobotPose);

        if (MetaConstants.updateQuestNav && questNav.isConnected()) PoseEstimatorConstants.questToWorldTransform = new Transform3d(lastQuestNavPose, new Pose3d(newRobotPose));
    }

    public void resetRobotPose() {
        DriveSubStateManager.getInstance().resetGyro(new Rotation2d());
        swerveDrivePoseEstimator.resetPosition(DriveSubStateManager.getInstance().getGyroAngleRotation2d(),
            DriveSubStateManager.getInstance().getModulePositions(), new Pose2d());

        
        if (MetaConstants.updateQuestNav && questNav.isConnected()) PoseEstimatorConstants.questToWorldTransform = new Transform3d(lastQuestNavPose, new Pose3d());

        
    }

    public void updateAlliance() {
        
        setRobotPose(new AutonPoint(getRobotPose()).getAutonPoint());
        for(PhotonPoseEstimator photonPoseEstimator : this.photonPoseEstimators) {
            photonPoseEstimator.getFieldTags().setOrigin(PoseEstimatorConstants.originPosition);
        }
        LogIOInputs.logToStateTable(photonPoseEstimators[0].getFieldTags().getOrigin(), "Vision/OrginPosition");
    }

    private void configPhotonPoseEstimators() {
        this.photonPoseEstimators[0] = new PhotonPoseEstimator(PoseEstimatorConstants.aprilTagLayout, PoseStrategy.LOWEST_AMBIGUITY, PoseEstimatorConstants.frontRightCameraToCenter);
        this.photonPoseEstimators[1] = new PhotonPoseEstimator(PoseEstimatorConstants.aprilTagLayout, PoseStrategy.LOWEST_AMBIGUITY, PoseEstimatorConstants.frontLeftCameraToCenter);
    }


    private void addVisionMeasurments() {
        if(MetaConstants.updateVision) {
            for (int i = 0; i < photonPoseEstimators.length; i++) {
                PhotonCamera camera = photonCameras[i];
                PhotonPoseEstimator estimator = photonPoseEstimators[i];

                List<PhotonPipelineResult> results = camera.getAllUnreadResults();
                if (results.isEmpty()) {
                    continue;
                }

                PhotonPipelineResult bestResult = null;
                PhotonTrackedTarget bestTarget = null;

                for (PhotonPipelineResult result : results) {
                    for (PhotonTrackedTarget target : result.getTargets()) {
                        boolean excluded = false;
                        for(int id : MetaConstants.isBlueAlliance ? PoseEstimatorConstants.tagWhitelistBlue : PoseEstimatorConstants.tagWhitelistRed) {
                            if(id == target.fiducialId) {
                                excluded = true;
                                continue;
                            }
                        }
                        if ((bestTarget == null ||
                            target.poseAmbiguity < bestTarget.poseAmbiguity)
                            && target.poseAmbiguity < PoseEstimatorConstants.maxAmbiguity && !excluded){

                            bestTarget = target;
                            bestResult = result;
                        }
                    }
                }
                if (bestResult != null) {
                    Optional<EstimatedRobotPose> estimatedPosition3d = estimator.update(bestResult);
                    if(estimatedPosition3d.isPresent()) {
                        double[][] tagFudgeOffsets = PoseEstimatorConstants.tagFudgeOffsets.get(camera.getName());
                        int id = bestTarget.fiducialId - 1;
                        double xFudge = MetaConstants.isBlueAlliance ?
                        tagFudgeOffsets[id][0] : tagFudgeOffsets[id][3];
                    
                        double yFudge = MetaConstants.isBlueAlliance ?
                        tagFudgeOffsets[id][1] : tagFudgeOffsets[id][4];

                        double rotFudge = MetaConstants.isBlueAlliance ? 
                        tagFudgeOffsets[id][2] : tagFudgeOffsets[id][5];
                        EstimatedRobotPose estimatedPose = estimatedPosition3d.get();
                        Pose2d estimatedPose2d = estimatedPose.estimatedPose.toPose2d();
                        Pose2d fudgedPosed = new Pose2d(estimatedPose2d.getX() + xFudge, estimatedPose2d.getY() + yFudge, Rotation2d.fromDegrees(estimatedPose2d.getRotation().getDegrees() + rotFudge));
                        if(!DriverStation.isEnabled() || ConversionUtils.getIsInTolerance(fudgedPosed, estimatedPose2d,
                            PoseEstimatorConstants.maxPoseDeltaFromCurrent)) {
                            swerveDrivePoseEstimator.addVisionMeasurement(fudgedPosed, estimatedPose.timestampSeconds, PoseEstimatorConstants.visionPoseEstimateTrust);
                        }
                    }
                }
            }
            
        }
        if(MetaConstants.updateQuestNav) { // && questNav.isTracking() TODO: THIS WILL CHANGE ON NEXT QUESTNAV UPDATE! isTracking will become per-frame instead of a method on questNav.
            // Get the latest pose data frames from the Quest
            try {
            PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();
            LogIOInputs.logToStateTable(questFrames.length > 0, "QuestNav/HasFrames");
            LogIOInputs.logToStateTable(questNav.getBatteryPercent(), "QuestNav/Battery");


            // Loop over the pose data frames and send them to the pose estimator
            for (PoseFrame questFrame : questFrames) {
                LogIOInputs.logToStateTable(questFrames.length > 0, "QuestNav/HasFrames");

                    Pose3d questPose = questFrame.questPose3d();

                    lastQuestNavPose = questPose;

                    double timestamp = questFrame.dataTimestamp();

                    // Transform by the mount pose to get your robot pose
                    Pose3d robotPose = questPose.transformBy(PoseEstimatorConstants.robotToQuestOffset.inverse()).transformBy(PoseEstimatorConstants.questToWorldTransform);

                    // Add the measurement to our estimator
                    LogIOInputs.logToStateTable(robotPose, "QuestNav/RobotPose");
                    LogIOInputs.logToStateTable(timestamp, "QuestNav/LastUpdateTimestamp");
                    
            if (Double.isNaN(swerveDrivePoseEstimator.getEstimatedPosition().getX())) { // TODO: This is a band-aid fix, figure out why NaN is occurring if possible
                swerveDrivePoseEstimator.resetPosition(DriveSubStateManager.getInstance().getGyroAngleRotation2d(),
                DriveSubStateManager.getInstance().getModulePositions(), robotPose.toPose2d());
            }
                    swerveDrivePoseEstimator.addVisionMeasurement(robotPose.toPose2d(), timestamp, PoseEstimatorConstants.questNavPoseEstimateTrust);
                }
                LogIOInputs.logToStateTable(true, "QuestNav/Updating");
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        } else {
            LogIOInputs.logToStateTable(false, "QuestNav/HasFrames");
            LogIOInputs.logToStateTable(false, "QuestNav/Updating");
        }
        
    
    }
}
/*Pose2d estimatedPosition = visionReading.estimatedPose.toPose2d();
                        // Pose2d estimatedPositionWithGyroAngle = new Pose2d(estimatedPosition.getTranslation(),
                        //    this.driveSubsystem.getGyroAngleRotation2d());
                        double xFudge = MetaConstants.isBlueAlliance ? 
                        PoseEstimatorConstants.tagFudgeOffsets[id][0] : PoseEstimatorConstants.tagFudgeOffsets[id][3];
                        
                        double yFudge = MetaConstants.isBlueAlliance ? 
                        PoseEstimatorConstants.tagFudgeOffsets[id][1] : PoseEstimatorConstants.tagFudgeOffsets[id][4];

                        double rotFudge = MetaConstants.isBlueAlliance ? 
                        PoseEstimatorConstants.tagFudgeOffsets[id][2] : PoseEstimatorConstants.tagFudgeOffsets[id][5];

                        Pose2d point = new Pose2d(estimatedPosition.getX() + xFudge, estimatedPosition.getY() + yFudge, 
                            estimatedPosition.getRotation().plus(Rotation2d.fromDegrees(rotFudge))); */