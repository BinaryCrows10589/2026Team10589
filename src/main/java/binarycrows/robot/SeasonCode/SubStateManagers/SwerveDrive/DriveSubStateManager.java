package binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive;

import static binarycrows.robot.SubStateManager.instance;

import java.util.ArrayList;
import java.util.HashMap;

import org.photonvision.PhotonPoseEstimator;

import binarycrows.robot.Keybinds;
import binarycrows.robot.PoseEstimator;
import binarycrows.robot.StateRequest;
import binarycrows.robot.StateTable;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.Enums.StateRequestStatus;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import binarycrows.robot.SeasonCode.Constants.VisionConstants;
import binarycrows.robot.Utils.LogIOInputs;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

public class DriveSubStateManager extends SubStateManager<DriveStateRequest> {
    
    private final SwerveModule frontLeftSwerveModule;
    private final SwerveModule frontRightSwerveModule;
    private final SwerveModule backLeftSwerveModule;
    private final SwerveModule backRightSwerveModule;
    
    public final GyroPigeonIO gyroPigeonIO;

    public SwerveModuleState[] swerveModuleStates;

    private ChassisSpeeds desiredChassisSpeeds;

    private PoseEstimator poseEstimator;

    private double currentVoltageTableTargetValue = 0;
    private double voltageTableStep = 0.1;
    private double voltageTableRecordingTime = 2;
    private double voltageTableMaxValue = SwerveDriveConstants.maxDriveMotorVoltage;
    private double voltageTableMinValue = 0;
    private boolean startedRecordingDeceleration = false;
    private ArrayList<Double> accelerationTable = new ArrayList<Double>();
    private ArrayList<Double> decelerationTable = new ArrayList<Double>();

    private Timer voltageRecordingTimer = new Timer();

    public final double driveDistance = 4;
    public double startDriveDistance = 0;
    public boolean hasStartedDrivingDistance = false;


    public DriveSubStateManager() {
        super();
        gyroPigeonIO = new GyroPigeonIO();
        frontLeftSwerveModule = new SwerveModule(new SwerveModuleTalonFXIO("FrontLeftModule"), "FrontLeftModule");
        frontRightSwerveModule = new SwerveModule(new SwerveModuleTalonFXIO("FrontRightModule"), "FrontRightModule");
        backLeftSwerveModule = new SwerveModule(new SwerveModuleTalonFXIO("BackLeftModule"), "BackLeftModule");
        backRightSwerveModule = new SwerveModule(new SwerveModuleTalonFXIO("BackRightModule"), "BackRightModule");

        poseEstimator = new PoseEstimator(gyroPigeonIO.yawAngle, getModulePositions());

        super.defaultState = new StateRequest<DriveStateRequest>(DriveStateRequest.DISABLE, StateRequestPriority.LOW);
    }

    private void recordVoltageTableValue() {
        ArrayList<Double> targetArray = (startedRecordingDeceleration ? decelerationTable : accelerationTable);
        targetArray.add(currentVoltageTableTargetValue);
        targetArray.add(frontLeftSwerveModule.getModuleState().speedMetersPerSecond);
    }

    @Override
    public void periodic() {
        super.periodic();

        gyroPigeonIO.update();
        poseEstimator.periodic();

        swerveModuleStates = getModuleStates();

        
        if (swerveModuleStates != null)
            LogIOInputs.logToStateTable(swerveModuleStates, "DriveSubsystem/ModuleStates");

        LogIOInputs.logToStateTable(frontLeftSwerveModule.getAbsoluteEncoderPosition(), "DriveSubsystem/FLAbsoluteEncoderPos");
        LogIOInputs.logToStateTable(frontRightSwerveModule.getAbsoluteEncoderPosition(), "DriveSubsystem/FRAbsoluteEncoderPos");
        LogIOInputs.logToStateTable(backLeftSwerveModule.getAbsoluteEncoderPosition(), "DriveSubsystem/BLAbsoluteEncoderPos");
        LogIOInputs.logToStateTable(backRightSwerveModule.getAbsoluteEncoderPosition(), "DriveSubsystem/BRAbsoluteEncoderPos");

        LogIOInputs.logToStateTable(frontLeftSwerveModule.getRelativeEncoderPosition(), "DriveSubsystem/FLRelativeEncoderPos");
        LogIOInputs.logToStateTable(frontRightSwerveModule.getRelativeEncoderPosition(), "DriveSubsystem/FRRelativeEncoderPos");
        LogIOInputs.logToStateTable(backLeftSwerveModule.getRelativeEncoderPosition(), "DriveSubsystem/BLRelativeEncoderPos");
        LogIOInputs.logToStateTable(backRightSwerveModule.getRelativeEncoderPosition(), "DriveSubsystem/BRRelativeEncoderPos");

        // Resolve pending state request
        if (this.activeStateRequest.getStatus() == StateRequestStatus.PENDING) {
            switch (this.activeStateRequest.getStateRequestType()) {
                case CONSTRUCT_VOLTAGE_TABLE:
                    voltageRecordingTimer.reset();
                    voltageRecordingTimer.start();
                    this.activeStateRequest.updateStatus(StateRequestStatus.RUNNING);
                    break;
                case TELEOP_DRIVE:
                    this.activeStateRequest.updateStatus(StateRequestStatus.RUNNING);
                default:
                    frontLeftSwerveModule.stopModuleDrive();
                    this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                    break;
            }
            

        }
        switch (this.activeStateRequest.getStateRequestType()) {
            case CONSTRUCT_VOLTAGE_TABLE:

            

                LogIOInputs.logToStateTable(currentVoltageTableTargetValue, "DriveSubsystem/VoltageTableTargetValue");
                LogIOInputs.logToStateTable(voltageRecordingTimer.get(), "DriveSubsystem/VoltageRecordingTimeElapsed");
                LogIOInputs.logToStateTable(accelerationTable, "DriveSubsystem/VoltageAccelerationTable");

                if (voltageRecordingTimer.hasElapsed(voltageTableRecordingTime)) { // It's time to record, buster brown!
                    System.out.println("Record new value");


                    if (!startedRecordingDeceleration) { // Still accelerating

                        if (currentVoltageTableTargetValue >= voltageTableMaxValue) { // Start bringing the speed back down since we've hit the max
                            startedRecordingDeceleration = true;
                            recordVoltageTableValue();
                        }

                        else { // Record the next value we need
                            recordVoltageTableValue();
                            currentVoltageTableTargetValue += voltageTableStep;
                            frontLeftSwerveModule.setDesiredModuleDriveVoltage(currentVoltageTableTargetValue);
                            voltageRecordingTimer.reset();
                        }
                    }
                    else { // Now decelerating
                        if (currentVoltageTableTargetValue <= voltageTableMinValue) { // Stop since we've hit the minimum
                            startedRecordingDeceleration = false;
                            this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                            this.returnToDefaultState();
                            recordVoltageTableValue();
                            LogIOInputs.logToStateTable(accelerationTable, "DriveSubsystem/AccelerationTable");
                            LogIOInputs.logToStateTable(decelerationTable, "DriveSubsystem/DecelerationTable");
                        }
                        else { // Record the next value we need
                            recordVoltageTableValue();
                            currentVoltageTableTargetValue -= voltageTableStep;
                            frontLeftSwerveModule.setDesiredModuleDriveVoltage(currentVoltageTableTargetValue);
                            voltageRecordingTimer.reset();
                        }
                    }
                }

                
                break;
                case TELEOP_DRIVE:
                    //driveOneTest();
                    //drivePeriodic();
                    break;
                case DRIVE_DISTANCE_TEST:
                    if (!hasStartedDrivingDistance) {
                        hasStartedDrivingDistance = true;
                        startDriveDistance = getDriveDistance();
                        //this.drive(0.1, 0, 0, false);
                    }
                    if (getDriveDistanceTotal() >= driveDistance) {
                        this.drive(0, 0, 0, true);
                    
                    } else {
                        this.drive(0.1, 0, 0, true);
                    }
                    LogIOInputs.logToStateTable(getDriveDistanceTotal(), "DriveSubsystem/DriveDistance");

                    break;
                default:
                break;
        }

    }

    private double getDriveDistanceTotal() {
        return frontLeftSwerveModule.getModuleDriveDistance() - startDriveDistance;
    }
    private double getDriveDistance() {
        return frontLeftSwerveModule.getModuleDriveDistance();
    }

    private double translationMax = SwerveDriveConstants.maxSpeedMetersPerSecond;
    private boolean normalizeTranslationMaximum = false;
    private double rotationMax = SwerveDriveConstants.maxRotationAnglePerSecond;

    @Override
    public Class<DriveStateRequest> getStateRequestType() {
        return DriveStateRequest.class;
    }


    // TODO:(ELIJAH) This should probaby be written better and less jankly then it is now(i know i wrote this)
    private void drivePeriodic() {
        if(StateTable.getValueAsBoolean("IsDriverControlled")) {
            boolean slowMode = StateTable.getValueAsBoolean("SlowMode");
            double translationX = slowMode ? 
            Keybinds.getTranslationX() * SwerveDriveConstants.maxSpeedMetersPerSecond * SwerveDriveConstants.translationXSlowModeMultipler :
            Keybinds.getTranslationX() * SwerveDriveConstants.maxSpeedMetersPerSecond;
            double translationY = slowMode ? 
            Keybinds.getTranslationY() * SwerveDriveConstants.maxSpeedMetersPerSecond * SwerveDriveConstants.translationYSlowModeMultipler : 
            Keybinds.getTranslationY() * SwerveDriveConstants.maxSpeedMetersPerSecond; 
            double rotation = slowMode ? 
            Keybinds.getRotation() * SwerveDriveConstants.maxRotationAnglePerSecond * SwerveDriveConstants.rotationSlowModeMultipler : 
            Keybinds.getRotation() * SwerveDriveConstants.maxRotationAnglePerSecond;

            LogIOInputs.logToStateTable(translationX, "DriveSubsystem/TranslationX");
            LogIOInputs.logToStateTable(translationY, "DriveSubsystem/TranslationY");
            LogIOInputs.logToStateTable(rotation, "DriveSubsystem/rotation");

            translationMax = SwerveDriveConstants.maxSpeedMetersPerSecond;
            rotationMax = SwerveDriveConstants.maxRotationAnglePerSecond;
            normalizeTranslationMaximum = false;

            double translationXSpeed;
            double translationYSpeed;
            
            if (normalizeTranslationMaximum) {
                double normalizedSpeed = Math.sqrt(Math.pow(translationX, 2) + Math.pow(translationY, 2));
                if(normalizedSpeed != 0) {
                    double translationFactor = MathUtil.clamp(normalizedSpeed, -translationMax, translationMax) / normalizedSpeed;
                    translationXSpeed = translationX * translationFactor;
                    translationYSpeed = translationY * translationFactor;
                } else {
                    translationXSpeed = MathUtil.clamp(translationX, -translationMax, translationMax);
                    translationYSpeed = MathUtil.clamp(translationY, -translationMax, translationMax);
                }
               
            } else { 
                translationXSpeed = MathUtil.clamp(translationX, -translationMax, translationMax);
                translationYSpeed = MathUtil.clamp(translationY, -translationMax, translationMax);
           } 
            double rotationSpeed = MathUtil.clamp(rotation, -rotationMax, rotationMax);


            if(StateTable.getValueAsBoolean("AxisLock")) {
                drive(-translationXSpeed, -translationYSpeed, rotationSpeed, false);
            } else {
                drive(translationXSpeed, translationYSpeed, rotationSpeed,true);
            }
        }
    }
    public void driveOneTest() {
        setModuleStates(new SwerveModuleState[] { 
            new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)), 
            new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)), 
            new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)), 
            new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)) 
        });
    }

    // TODO:(ELIJAH) Bring out the if...setModuleStates to the periodic directly. This will make the drive(chassisSpeeds) work as well
    public void drive(double desiredXVelocity, double desiredYVelocity, double desiredRotationalVelocity, boolean isFieldRelative) {
        this.desiredChassisSpeeds = isFieldRelative ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(desiredXVelocity, desiredYVelocity,
                desiredRotationalVelocity, this.poseEstimator.getRobotPose().getRotation()) :
            new ChassisSpeeds(desiredXVelocity, desiredYVelocity,
                desiredRotationalVelocity);

            if (desiredChassisSpeeds != null) {  
                SwerveModuleState[] desiredStates = SwerveDriveConstants.driveKinematics.toSwerveModuleStates(desiredChassisSpeeds);   
                // If we're not trying to move, we lock the angles of the wheels
                if (desiredChassisSpeeds.vxMetersPerSecond == 0.0 && desiredChassisSpeeds.vyMetersPerSecond == 0.0
                    && desiredChassisSpeeds.omegaRadiansPerSecond == 0.0) {
                    SwerveModuleState[] currentStates = swerveModuleStates;
                    for(int i = 0; i < currentStates.length; i++) {
                        desiredStates[i].angle = currentStates[i].angle;
                    }
                }
                setModuleStates(desiredStates);
            }
            // Resets the desiredChassisSpeeds to null to stop it from "sticking" to the last states
            desiredChassisSpeeds = null;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        LogIOInputs.logToStateTable(desiredStates, "DriveSubsystem/DesiredModuleStates");
        SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveDriveConstants.maxSpeedMetersPerSecond);
        this.frontLeftSwerveModule.setDesiredModuleState(desiredStates[0]);
        this.frontRightSwerveModule.setDesiredModuleState(desiredStates[1]);
        this.backLeftSwerveModule.setDesiredModuleState(desiredStates[2]);
        this.backRightSwerveModule.setDesiredModuleState(desiredStates[3]); 
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = {
            this.frontLeftSwerveModule.getModulePosition(),
            this.frontRightSwerveModule.getModulePosition(),
            this.backLeftSwerveModule.getModulePosition(),
            this.backRightSwerveModule.getModulePosition()
        };
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = {
            this.frontLeftSwerveModule.getModuleState(),
            this.frontRightSwerveModule.getModuleState(),
            this.backLeftSwerveModule.getModuleState(),
            this.backRightSwerveModule.getModuleState()
        };
        return states;
    }

    public void drive(ChassisSpeeds desiredChassisSpeeds) {
        this.desiredChassisSpeeds = desiredChassisSpeeds;
    }

    public void resetGyro() {
        gyroPigeonIO.resetAngle(Rotation2d.kZero);
    }
    public void resetGyro(Rotation2d rotation) {
        gyroPigeonIO.resetAngle(rotation);
    }

    public static DriveSubStateManager getInstance() {
        assert instance != null;
        return (DriveSubStateManager) instance;
    }

    public Rotation2d getGyroAngleRotation2d() {
        return gyroPigeonIO.yawAngle;
    }

    @Override
    public String toString() {
        return "Drive SubState Manager";
    }

    
    //TODO:(Elijah) To fix this simply move it to a pose estimator state manager and has all of the vision stuff. I can help you impliment the vision state manager if needed
    public void updateAlliance() { //TODO: Make this not need to be disabled...
        
        //if(MetaConstants.hasAllianceChanged) {
            //AprilTagFieldLayout fieldTags = photonPoseEstimators[0].getFieldTags();
            //if(MetaConstants.isBlueAlliance) {
            //    fieldTags.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            //    VisionConstants.originPosition = OriginPosition.kBlueAllianceWallRightSide;
            //    VisionConstants.kExcludedTags = VisionConstants.kExcludedTagsBlue;
            //} else {
            //    fieldTags.setOrigin(OriginPosition.kRedAllianceWallRightSide);
            //    VisionConstants.originPosition = OriginPosition.kRedAllianceWallRightSide;
            //    VisionConstants.kExcludedTags = VisionConstants.kExcludedTagsRed;
            //}
            //TODO: Dispabled for crowmotion testing
            //setRobotPose(flipAlliance(getRobotPose()));
            //for(PhotonPoseEstimator photonPoseEstimator : this.photonPoseEstimators) {
            //    photonPoseEstimator.getFieldTags().setOrigin(VisionConstants.originPosition);
            //    Logger.recordOutput("Vision/OrginPosition", photonPoseEstimator.getFieldTags().getOrigin());
            //}
            //Logger.recordOutput("Vision/OrginPosition", photonPoseEstimators[0].getFieldTags().getOrigin());
        //}    
    }
}
