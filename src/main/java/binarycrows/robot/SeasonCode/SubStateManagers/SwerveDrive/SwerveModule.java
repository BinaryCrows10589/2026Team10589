package binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import binarycrows.robot.Utils.ConversionUtils;
import binarycrows.robot.Utils.LogIOInputs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private final SwerveModuleTalonFXIO swerveModuleIO;
    private String swerveModuleName = "NoModuleNameSet";

    //private SwerveDriveVoltageVSMetersPerSecondTableCreater voltageTableCreator;

    /**
     * Creates a Swerve Module Object and compleates all initialization
     * @param swerveModuleIO SwerveModuleIO: The IO object for the Swerve Module. It can be either SwerveModule
     * @param swerveModuleName
     */
    public SwerveModule(SwerveModuleTalonFXIO swerveModuleIO, String swerveModuleName) {
        this.swerveModuleIO = swerveModuleIO;
        this.swerveModuleName = swerveModuleName;
    }

    /**
     * Sets the desired state of the Swerve Module
     * @param desiredState SwerveModuleState: The desired state for the module
     */
    public void setDesiredModuleState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = ConversionUtils.optimizeSwerveModuleState(desiredState, getModuleState().angle); 
        
        // Temporary option for drive voltage
        double driveVoltage = optimizedState.speedMetersPerSecond == 0 ? 0 : 
        MathUtil.clamp((((optimizedState.speedMetersPerSecond/SwerveDriveConstants.maxSpeedMPS) * 13)
        + (SwerveDriveConstants.driveFeedForward * Math.signum(optimizedState.speedMetersPerSecond))), -13, 13);
        this.swerveModuleIO.setDesiredModuleDriveVoltage(driveVoltage);
        this.swerveModuleIO.setDesiredModuleAngle(optimizedState.angle);

        // The other option we need to implement
        //double desiredRPM = optimizedState.speedMetersPerSecond / SwerveModuleConstants.kDriveConversionVelocityFactor;
        //this.swerveModuleIO.setDesiredModuleVelocityRPM(desiredRPM);
        //this.swerveModuleIO.setDesiredModuleDriveVoltage(metersPerSecondToVoltage(optimizedState.speedMetersPerSecond));
    }

    public void setDesiredModuleDriveVoltage(double driveVoltage) {
        this.swerveModuleIO.setDesiredModuleDriveVoltage(driveVoltage);
    }

    public void resetTurningMotorToAbsolute() {
        swerveModuleIO.resetTurningMotorToAbsolute();
    }

    public Rotation2d getAbsoluteEncoderPosition() {
        return swerveModuleIO.getAbsoluteEncoderPosition();
    }
    public Rotation2d getRelativeEncoderPosition() {
        return Rotation2d.fromRotations(swerveModuleIO.getWheelAngleRelativePositionRotations());
    }


    // TODO: REIMPLEMENT THIS AS A LERP TABLE!
    public double metersPerSecondToVoltage(double desiredMetersPerSecond) {
        double percentOfMaxSpeed = desiredMetersPerSecond / SwerveDriveConstants.maxSpeedMPS;
        double appliedFeedforwardVoltage = (desiredMetersPerSecond != 0 ? SwerveDriveConstants.voltageFeedForward : 0) *
            Math.signum(desiredMetersPerSecond);
        double unclampedVoltage = percentOfMaxSpeed *
            SwerveDriveConstants.voltageForMaxSpeed +
            appliedFeedforwardVoltage;
        double clampedVoltage = MathUtil.clamp(unclampedVoltage, -SwerveDriveConstants.voltageForMaxSpeed, SwerveDriveConstants.voltageForMaxSpeed);
        return clampedVoltage;
    }

    public void stopModuleDrive() {
        this.swerveModuleIO.setDesiredModuleDriveVoltage(0);
    }
    
    /**
     * Gets the position of the module
     * @return SwerveModulePosition: The Swerve Module Position, usefull for odometry
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            swerveModuleIO.getDriveMotorDistance(), Rotation2d.fromRotations(swerveModuleIO.getWheelAngleRelativePositionRotations()));
    }

    public double getModuleDriveDistance() {
        return swerveModuleIO.getDriveMotorDistance() * SwerveDriveConstants.driveConversionPositionFactor;
    }

    /**
     * The current state of the module
     * @return SwerveModuleState: The current state of the module
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveMotorSpeedInMetersPerSecond(), getWheelRotationAsRotation2d());
    }

    /**
     * The Speed of the drive motor in meters per second
     * @return Double: The current speed of the drive motor in MPS(Meters per second)
     */
    private double getDriveMotorSpeedInMetersPerSecond() {
        return this.swerveModuleIO.getDriveMotorRPS() * 60 * SwerveDriveConstants.driveConversionVelocityFactor;
    }

    /**
     * The current angle of the module's wheel
     * @return Rotation2d: The module's wheel angle
     */
    private Rotation2d getWheelRotationAsRotation2d() {
        return Rotation2d.fromRotations(swerveModuleIO.getWheelAngleRelativePositionRotations());
    }
}
