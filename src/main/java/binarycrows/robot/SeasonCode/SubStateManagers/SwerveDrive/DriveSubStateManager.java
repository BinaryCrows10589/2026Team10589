package binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive;

import java.util.ArrayList;
import java.util.HashMap;

import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import binarycrows.robot.Keybinds;
import binarycrows.robot.MainStateManager;
import binarycrows.robot.PoseEstimator;
import binarycrows.robot.StateRequest;
import binarycrows.robot.StateTable;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.CrowMotion.UserSide.CMAutonPoint;
import binarycrows.robot.CrowMotion.UserSide.CMRotation;
import binarycrows.robot.CrowMotion.UserSide.CMStateRequest;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory.TrajectoryPriority;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.Enums.StateRequestStatus;
import binarycrows.robot.SeasonCode.Constants.CrowMotionConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import binarycrows.robot.SeasonCode.Constants.PoseEstimatorConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.GyroIO.GyroOutputs;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.SwerveModuleIO.SwerveModuleOutputs;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretSubStateManager;
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

    public SwerveModuleOutputs frontLeftOutputs;
    public SwerveModuleOutputs frontRightOutputs;
    public SwerveModuleOutputs backLeftOutputs;
    public SwerveModuleOutputs backRightOutputs;
    
    public final GyroIO gyroIO;

    public final GyroOutputs gyroOutputs;

    public SwerveModuleState[] swerveModuleStates;

    private ChassisSpeeds desiredChassisSpeeds;

    private PoseEstimator poseEstimator;

    private double currentVoltageTableTargetValue = 0;
    private double voltageTableStep = 0.0002;
    private ArrayList<Double> voltage = new ArrayList<Double>();
    private ArrayList<Double> velocityFL = new ArrayList<Double>();
    private ArrayList<Double> velocityFR = new ArrayList<Double>();
    private ArrayList<Double> velocityBL = new ArrayList<Double>();
    private ArrayList<Double> velocityBR = new ArrayList<Double>();
    private int consecutiveMovement = 0;
    private final int maxConsecutiveMovement = 100;
    private int trajectoryArrayIndex = 0;

    private Timer voltageRecordingTimer = new Timer();

    public final double driveDistance = 4;
    public double startDriveDistance = 0;
    public boolean hasStartedDrivingDistance = false;
  
    public long trajectoryStartTime = -1;


    public DriveSubStateManager() {
        super();

        gyroOutputs = new GyroOutputs();
        frontLeftOutputs = new SwerveModuleOutputs();
        frontRightOutputs = new SwerveModuleOutputs();
        backLeftOutputs = new SwerveModuleOutputs();
        backRightOutputs = new SwerveModuleOutputs();

        gyroIO = MetaConstants.isReal ? new GyroPigeonIO(gyroOutputs) : new GyroSimIO(gyroOutputs);
        frontLeftSwerveModule = constructSwerveModule("FrontLeftModule", frontLeftOutputs);
        frontRightSwerveModule = constructSwerveModule("FrontRightModule", frontRightOutputs);
        backLeftSwerveModule = constructSwerveModule("BackLeftModule", backLeftOutputs);
        backRightSwerveModule = constructSwerveModule("BackRightModule", backRightOutputs);

        poseEstimator = new PoseEstimator(gyroOutputs.yawAngle, getModulePositions());

        super.defaultState = new StateRequest<DriveStateRequest>(DriveStateRequest.TELEOP_DRIVE, StateRequestPriority.NORMAL);
    }
    private SwerveModule constructSwerveModule(String name, SwerveModuleOutputs outputs) {
        return new SwerveModule(MetaConstants.isReal ? new SwerveModuleTalonFX(name, outputs) : new SwerveModuleSim(name, outputs), name);
    }

    private void recordVoltageTableValue() {
        voltage.add(currentVoltageTableTargetValue);
        velocityFL.add(frontLeftSwerveModule.getModuleState().speedMetersPerSecond);
        velocityFR.add(frontRightSwerveModule.getModuleState().speedMetersPerSecond);
        velocityBL.add(backLeftSwerveModule.getModuleState().speedMetersPerSecond);
        velocityBR.add(backRightSwerveModule.getModuleState().speedMetersPerSecond);
    }

    public void updatePoseEstimatorAlliance() {
        poseEstimator.updateAlliance();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return SwerveDriveConstants.driveKinematics.toChassisSpeeds(this.swerveModuleStates);
    }

    public void setRobotPose(Pose2d newRobotPose) {
        this.poseEstimator.setRobotPose(newRobotPose);
    }

    public void setRobotStartingPose(Pose2d newRobotPose) {
        setRobotPose(newRobotPose);
    }

    public void enableSlowMode() {
        StateTable.putValue("SlowMode", true);
    }
    public void disableSlowMode() {
        StateTable.putValue("SlowMode", false);
    }

    public void enableForceRobotRelative() {
        StateTable.putValue("ForceRobotRelative", true);
    }
    public void disableForceRobotRelative() {
        StateTable.putValue("ForceRobotRelative", false);
    }

    @Override
    public void recieveStateRequest(StateRequest request) {



        // Initialize trajectory from CMStateRequest
        if (request instanceof CMStateRequest) {
            CrowMotionConstants.currentTrajectory = ((CMStateRequest)request).trajectory;
            CrowMotionConstants.currentTrajectory.init();
        }

        if (request.getStateRequestType() == DriveStateRequest.DRIVE_CROWMOTION) {
            StateTable.putValue("IsDriverControlled", false);

            if (CrowMotionConstants.currentTrajectory == null || CrowMotionConstants.currentTrajectory.isCompleted()) {
                request.updateStatus(StateRequestStatus.REJECTED);
                LogIOInputs.logStateRequestRejection(request, "currentTrajectory is null or already completed");
                return;
            }
        } else if (request.getStateRequestType() == DriveStateRequest.DRIVE_CROWMOTION_ARRAY) {
            StateTable.putValue("IsDriverControlled", false);

            if (CrowMotionConstants.currentTrajectoryArray == null) {
                request.updateStatus(StateRequestStatus.REJECTED);
                LogIOInputs.logStateRequestRejection(request, "currentTrajectoryArray is null");
                return;
            }
        } else if (request.getStateRequestType() == DriveStateRequest.DRIVE_CROWMOTION_AUTOPOSITIONING) {
            StateTable.putValue("IsDriverControlled", false);
            Pose2d currentRobotPose = poseEstimator.getRobotPose();

            CrowMotionConstants.currentTrajectory = new CMTrajectory(
                "TestTraj1",
                    new CMAutonPoint[] {
                        new CMAutonPoint(4, -2, false)
                    }, 
                    new CMRotation[] {
                        new CMRotation(-140, 0, 1, 240, 240, 240, 3, false)
                    }, 
                        null,
                        4,
                        8,
                        TrajectoryPriority.SPLIT_PROPORTIONALLY,
                        2.5,
                        3,
                        3,
                        Math.sqrt(getChassisSpeeds().vxMetersPerSecond * getChassisSpeeds().vxMetersPerSecond + 
                        getChassisSpeeds().vyMetersPerSecond * getChassisSpeeds().vyMetersPerSecond),
                        true,
                        .05,
                        .02,
                        new double[] {.02, .02}, .04, 50);
            request = new StateRequest<DriveStateRequest>(DriveStateRequest.DRIVE_CROWMOTION, request.getPriority());
        }
        else if (request.getStateRequestType() == DriveStateRequest.TELEOP_DRIVE) {
            StateTable.putValue("IsDriverControlled", true);

        }
        super.recieveStateRequest(request);
    }

    private long startTimestamp = -1;
    private double maxSpeed = 0;

    @Override
    public void periodic() {
        super.periodic();

        swerveModuleStates = getModuleStates(); // Must be ran first because gyro sim is dependent on this value

        gyroIO.update();
        poseEstimator.periodic();

        frontLeftSwerveModule.update();
        frontRightSwerveModule.update();
        backLeftSwerveModule.update();
        backRightSwerveModule.update();

        LogIOInputs.logObjectToStateTable(frontLeftOutputs, "SwerveModule/FrontLeft");
        LogIOInputs.logObjectToStateTable(frontRightOutputs, "SwerveModule/FrontRight");
        LogIOInputs.logObjectToStateTable(backLeftOutputs, "SwerveModule/BackLeft");
        LogIOInputs.logObjectToStateTable(backRightOutputs, "SwerveModule/BackRight");


        
        if (swerveModuleStates != null)
            LogIOInputs.logToStateTable(swerveModuleStates, "DriveSubsystem/ModuleStates");

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
                    break;
                case DRIVE_CROWMOTION:
                    
                    trajectoryStartTime = System.currentTimeMillis();
                    CrowMotionConstants.currentTrajectory.init();
                    this.activeStateRequest.updateStatus(StateRequestStatus.RUNNING);
                    break;
                case DRIVE_CROWMOTION_ARRAY:
                    trajectoryArrayIndex = 0;
                    if (CrowMotionConstants.currentTrajectoryArray.length == 0) {
                        this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                        break;
                    }
                    CrowMotionConstants.currentTrajectoryArray[trajectoryArrayIndex].init();
                    CrowMotionConstants.currentTrajectory = CrowMotionConstants.currentTrajectoryArray[trajectoryArrayIndex];

                    this.activeStateRequest.updateStatus(StateRequestStatus.RUNNING);
                    break;
                default:
                    frontLeftSwerveModule.stopModuleDrive();
                    this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                    break;
            }

        }
        if (this.activeStateRequest.getStatus() == StateRequestStatus.RUNNING) {
            switch (this.activeStateRequest.getStateRequestType()) {
                case CONSTRUCT_VOLTAGE_TABLE:

                

                    LogIOInputs.logToStateTable(currentVoltageTableTargetValue, "DriveSubsystem/VoltageTableTargetValue");
                    LogIOInputs.logToStateTable(voltageRecordingTimer.get(), "DriveSubsystem/VoltageRecordingTimeElapsed");

                    if (consecutiveMovement > maxConsecutiveMovement) {
                        if (currentVoltageTableTargetValue <= 0) {
                            
                            this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                            this.returnToDefaultState();
                            recordVoltageTableValue();
                            LogIOInputs.logToStateTable(voltage, "DriveSubsystem/Voltage");
                        }
                        else {
                                recordVoltageTableValue();
                                currentVoltageTableTargetValue -= voltageTableStep;
                                System.out.println("Dec voltage to " + currentVoltageTableTargetValue);
                                frontLeftSwerveModule.setDesiredModuleDriveVoltage(currentVoltageTableTargetValue);
                                frontRightSwerveModule.setDesiredModuleDriveVoltage(currentVoltageTableTargetValue);
                                backLeftSwerveModule.setDesiredModuleDriveVoltage(currentVoltageTableTargetValue);
                                backRightSwerveModule.setDesiredModuleDriveVoltage(currentVoltageTableTargetValue);
                        }
                    }
                    else {
                        if (
                            frontLeftSwerveModule.getModuleState().speedMetersPerSecond != 0 &&
                            frontRightSwerveModule.getModuleState().speedMetersPerSecond != 0 &&
                            backLeftSwerveModule.getModuleState().speedMetersPerSecond != 0 &&
                            backRightSwerveModule.getModuleState().speedMetersPerSecond != 0
                            ) {
                            consecutiveMovement++;
                            System.out.println("Moving!");
                            
                        } else {
                            consecutiveMovement = 0;
                            recordVoltageTableValue();
                            currentVoltageTableTargetValue += voltageTableStep;
                            System.out.println("Inc voltage to " + currentVoltageTableTargetValue);
                            frontLeftSwerveModule.setDesiredModuleDriveVoltage(currentVoltageTableTargetValue);
                            frontRightSwerveModule.setDesiredModuleDriveVoltage(currentVoltageTableTargetValue);
                            backLeftSwerveModule.setDesiredModuleDriveVoltage(currentVoltageTableTargetValue);
                            backRightSwerveModule.setDesiredModuleDriveVoltage(currentVoltageTableTargetValue);
                        }
                    }

                    
                    break;
                    case TELEOP_DRIVE:
                        drivePeriodic();
                        ChassisSpeeds speeds = getChassisSpeeds();
                        double speed = Math.sqrt(speeds.vxMetersPerSecond * speeds.vxMetersPerSecond + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);
                        LogIOInputs.logToStateTable(speed, "speed");
                        if (speed > maxSpeed) maxSpeed = speed;

                        LogIOInputs.logToStateTable(maxSpeed, "maxSpeed");
                        if (startTimestamp == -1 && speed > 0.05) {
                            startTimestamp = System.currentTimeMillis();
                        }
                        else if (speed > SwerveDriveConstants.maxSpeedMPS - 0.05) {
                            LogIOInputs.logToStateTable((System.currentTimeMillis() - startTimestamp) /1000.0, "timeToReachMaxSpeedSec");
                        }
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
                    case DRIVE_CROWMOTION:
                        CrowMotionConstants.currentTrajectory.runTrajectoryFrame();
                        drivePeriodic();
                        if (CrowMotionConstants.currentTrajectory.isCompleted() && this.activeStateRequest.getStatus() != StateRequestStatus.FULFILLED) {
                            this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                            LogIOInputs.logToStateTable((System.currentTimeMillis() - trajectoryStartTime) / 1000.0, "DriveSubsystem/LastTrajectoryTimeSec");
                        }
                        break;
                    case DRIVE_CROWMOTION_ARRAY:
                        this.drive(.001,0, 0, true);

                        CrowMotionConstants.currentTrajectory.runTrajectoryFrame();
                        drivePeriodic();
                        if (CrowMotionConstants.currentTrajectory.isCompleted() && this.activeStateRequest.getStatus() != StateRequestStatus.FULFILLED) {
                            trajectoryArrayIndex++;
                            if (trajectoryArrayIndex >= CrowMotionConstants.currentTrajectoryArray.length) {
                                this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                            } else {
                                CrowMotionConstants.currentTrajectoryArray[trajectoryArrayIndex].init();
                                CrowMotionConstants.currentTrajectory = CrowMotionConstants.currentTrajectoryArray[trajectoryArrayIndex];
                            }
                        }
                        break;
                    
                    default:
                    break;
            }
        }

    }

    public Pose2d getRobotPose() {
        return poseEstimator.getRobotPose();
    }

    public double[] getRobotPoseCM() {
        Pose2d pose = poseEstimator.getRobotPose();
        return new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
    }
    public double[] getRobotVelocityCM() {
        ChassisSpeeds speeds = SwerveDriveConstants.driveKinematics.toChassisSpeeds(swerveModuleStates);
        return new double[] {speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, (speeds.omegaRadiansPerSecond * (180 / Math.PI))};
    }

    public void driveCM(double[] speeds) {
        this.drive(speeds[0], speeds[1],  ((Math.PI / 180) *  speeds[2]), true);
    }

    private double getDriveDistanceTotal() {
        return getDriveDistance() - startDriveDistance;
    }
    private double getDriveDistance() {
        return poseEstimator.getRobotPose().getY();
    }

    private double translationMax = SwerveDriveConstants.maxSpeedMPS;
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
            double[] translation = Keybinds.getTranslation();
            double translationX = slowMode ?
                translation[0] * SwerveDriveConstants.maxSpeedMPS * SwerveDriveConstants.translationXSlowModeMultipler :
                translation[0] * SwerveDriveConstants.maxSpeedMPS;
            double translationY = slowMode ?
                translation[1] * SwerveDriveConstants.maxSpeedMPS * SwerveDriveConstants.translationXSlowModeMultipler :
                translation[1] * SwerveDriveConstants.maxSpeedMPS;
            double rotation = slowMode ? 
            Keybinds.getRotation() * SwerveDriveConstants.maxRotationAnglePerSecond * SwerveDriveConstants.rotationSlowModeMultipler : 
            Keybinds.getRotation() * SwerveDriveConstants.maxRotationAnglePerSecond;

            LogIOInputs.logToStateTable(translationX, "DriveSubsystem/TranslationX");
            LogIOInputs.logToStateTable(translationY, "DriveSubsystem/TranslationY");
            LogIOInputs.logToStateTable(rotation, "DriveSubsystem/rotation");
            LogIOInputs.logToStateTable(Keybinds.getRotation(), "DriveSubsystem/rotationraw");

            translationMax = SwerveDriveConstants.maxSpeedMPS;
            rotationMax = SwerveDriveConstants.maxRotationAnglePerSecond;

            double translationXSpeed;
            double translationYSpeed;

            boolean normalizeTranslationMaximum = false;
            
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




            if(StateTable.getValueAsBoolean("ForceRobotRelative")) {
                drive(translationXSpeed, translationYSpeed, rotationSpeed, false);
            } else {
                drive(translationXSpeed, translationYSpeed, rotationSpeed,true);
            }
        }

        
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
            // Resets the desiredChassisSpeeds to null to stop it from "sticking" to the last states
            desiredChassisSpeeds = null;
        }
    }

    public void resetRobotPose() {
        poseEstimator.resetRobotPose();
    }

    public void drive(double desiredXVelocity, double desiredYVelocity, double desiredRotationalVelocity, boolean isFieldRelative) {
        LogIOInputs.logToStateTable(desiredXVelocity, "Driver/XVelocity");
        LogIOInputs.logToStateTable(desiredYVelocity, "Driver/YVelocity");
        LogIOInputs.logToStateTable(desiredRotationalVelocity, "Driver/RotVelocity");
        this.desiredChassisSpeeds = isFieldRelative ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(desiredXVelocity, desiredYVelocity,
                desiredRotationalVelocity, this.poseEstimator.getRobotPose().getRotation()) :
            new ChassisSpeeds(desiredXVelocity, desiredYVelocity,
                desiredRotationalVelocity);
        switch (Keybinds.getAxisLock()) {
            case 1:
            this.desiredChassisSpeeds.vxMetersPerSecond = 0;
            break;
            case 2:
            this.desiredChassisSpeeds.vyMetersPerSecond = 0;
            break;
            default:
            break;
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        LogIOInputs.logToStateTable(desiredStates, "DriveSubsystem/DesiredModuleStates");
        SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveDriveConstants.maxSpeedMPS);
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
        gyroIO.resetAngle(Rotation2d.kZero);
    }
    public void resetGyro(Rotation2d rotation) {
        gyroIO.resetAngle(rotation);
    }

    public Rotation2d getGyroAngleRotation2d() {
        return gyroOutputs.yawAngle;
    }

    @Override
    public String toString() {
        return "Drive SubState Manager";
    }

    public static DriveSubStateManager getInstance() {
        return (DriveSubStateManager) MainStateManager.getInstance().resolveSubStateManager(DriveStateRequest.class);
    } 
    
}
