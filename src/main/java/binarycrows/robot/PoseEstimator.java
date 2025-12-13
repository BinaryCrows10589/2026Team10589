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
import binarycrows.robot.Utils.LogIOInputs;
import binarycrows.robot.Utils.TolorenceUtil;
import binarycrows.robot.Utils.Auton.AutonPoint;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;

public class PoseEstimator {
    private PhotonCamera[] photonCameras = {new PhotonCamera("Unknown1"), new PhotonCamera("BRModuleCam")};
    private PhotonPoseEstimator[] photonPoseEstimators = new PhotonPoseEstimator[photonCameras.length];
    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    private Notifier visionNotifier = new Notifier(this::addVisionMeasurments);

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
        try {
            this.swerveDrivePoseEstimator.update(DriveSubStateManager.getInstance().gyroOutputs.yawAngle,
                DriveSubStateManager.getInstance().getModulePositions());

            LogIOInputs.logToStateTable(this.swerveDrivePoseEstimator.getEstimatedPosition(), "PoseEstimator/EstimatedPosition");

        } catch(Exception E) {
           System.out.println("FAILED TO UPDATE: " + E.getMessage());
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
    }

    public void resetRobotPose() {
        DriveSubStateManager.getInstance().resetGyro(new Rotation2d());
        swerveDrivePoseEstimator.resetPosition(DriveSubStateManager.getInstance().getGyroAngleRotation2d(),
            DriveSubStateManager.getInstance().getModulePositions(), new Pose2d());
        
    }

    public void updateAlliance() {
        DriveSubStateManager.getInstance().updateAlliance();
    }

    private void configPhotonPoseEstimators() {
        //this.photonPoseEstimators[0] = new PhotonPoseEstimator(VisionConstants.aprilTagLayout, PoseStrategy.LOWEST_AMBIGUITY, VisionConstants.kBackLeftCameraToCenter);
        //this.photonPoseEstimators[1] = new PhotonPoseEstimator(VisionConstants.aprilTagLayout, PoseStrategy.LOWEST_AMBIGUITY, VisionConstants.kBackRightCameraToCenter);
        System.out.println("Don't forget to configure the photon position estimators someday..."); //TODO
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
                        for(int id : PoseEstimatorConstants.kExcludedTags) {
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
                        int id = bestTarget.fiducialId;
                        double xFudge = MetaConstants.isBlueAlliance ? 
                        PoseEstimatorConstants.tagFudgeOffsets[id][0] : PoseEstimatorConstants.tagFudgeOffsets[id][3];
                    
                        double yFudge = MetaConstants.isBlueAlliance ? 
                        PoseEstimatorConstants.tagFudgeOffsets[id][1] : PoseEstimatorConstants.tagFudgeOffsets[id][4];

                        double rotFudge = MetaConstants.isBlueAlliance ? 
                        PoseEstimatorConstants.tagFudgeOffsets[id][2] : PoseEstimatorConstants.tagFudgeOffsets[id][5];
                        EstimatedRobotPose estimatedPose = estimatedPosition3d.get();
                        Pose2d estimatedPose2d = estimatedPose.estimatedPose.toPose2d();
                        Pose2d fudgedPosed = new Pose2d(estimatedPose2d.getX() + xFudge, estimatedPose2d.getY() + yFudge, Rotation2d.fromDegrees(estimatedPose2d.getRotation().getDegrees() + rotFudge));
                        if(!DriverStation.isEnabled() || TolorenceUtil.inPoseTolorence(fudgedPosed, estimatedPose2d,
                            PoseEstimatorConstants.maxPoseDeltaFromCurrent)) {
                            swerveDrivePoseEstimator.addVisionMeasurement(fudgedPosed, estimatedPose.timestampSeconds);
                        }
                    }
                }
            }
            
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