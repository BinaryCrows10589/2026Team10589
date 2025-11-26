package binarycrows.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import binarycrows.robot.SeasonCode.Constants.VisionConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveSubStateManager;
import binarycrows.robot.Utils.LogIOInputs;
import binarycrows.robot.Utils.Auton.AutonPoint;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class PoseEstimator {



    private PhotonCamera[] photonCameras = {new PhotonCamera("Unknown1"), new PhotonCamera("BRModuleCam")};
    private PhotonPoseEstimator[] photonPoseEstimators = new PhotonPoseEstimator[photonCameras.length];

    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    public PoseEstimator(Rotation2d yawAngle, SwerveModulePosition[] modulePositions) {
        
        configPhotonPoseEstimators();
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
            SwerveDriveConstants.driveKinematics,
            yawAngle,
            modulePositions, 
            new Pose2d(),
            VisionConstants.swerveDrivePoseEstimateTrust, VisionConstants.visionPoseEstimateTrust);
        VisionConstants.aprilTagLayout.setOrigin(VisionConstants.originPosition);

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
}
