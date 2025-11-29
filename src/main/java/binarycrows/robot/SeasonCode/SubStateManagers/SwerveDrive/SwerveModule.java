package binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive;


import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import binarycrows.robot.SeasonCode.Utils.DesiredMetersPerSecondToVoltageLerpTable;
import binarycrows.robot.Utils.ConversionUtils;
import binarycrows.robot.Utils.LogIOInputs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private final SwerveModuleIO swerveModuleIO;
    private String swerveModuleName = "NoModuleNameSet";

    /*public ArrayList<Double> voltageColumn = new ArrayList<Double>();
    public ArrayList<Double> velocityColumn = new ArrayList<Double>();*/


    //private SwerveDriveVoltageVSMetersPerSecondTableCreater voltageTableCreator;

    /**
     * Creates a Swerve Module Object and compleates all initialization
     * @param swerveModuleIO SwerveModuleIO: The IO object for the Swerve Module. It can be either SwerveModule
     * @param swerveModuleName
     */
    public SwerveModule(SwerveModuleIO swerveModuleIO, String swerveModuleName) {
        this.swerveModuleIO = swerveModuleIO;
        this.swerveModuleName = swerveModuleName;
    }

    public void update() {
        swerveModuleIO.update();
    }

    /**
     * Sets the desired state of the Swerve Module
     * @param desiredState SwerveModuleState: The desired state for the module
     */
    public void setDesiredModuleState(SwerveModuleState desiredState) {
        LogIOInputs.logToStateTable(desiredState.speedMetersPerSecond, swerveModuleName + "/DesiredSpeedMPS");

        LogIOInputs.logToStateTable(getDriveMotorSpeedInMetersPerSecond(), swerveModuleName + "/SpeedMPS");


        System.out.println(this.swerveModuleName + " desired module rotations: " + desiredState.angle.getRotations());
        System.out.println(this.swerveModuleName + " actual module rotations: " + getModuleState().angle.getRotations());
        SwerveModuleState optimizedState = ConversionUtils.optimizeSwerveModuleState(desiredState, getModuleState().angle); 
        System.out.println(this.swerveModuleName + " desired module rotations: " + optimizedState.angle.getRotations() + "(optimized)");

        if (Double.isNaN(optimizedState.speedMetersPerSecond)) optimizedState.speedMetersPerSecond = 0;
        
        double driveVoltage = metersPerSecondToVoltage(optimizedState.speedMetersPerSecond);
        
        //metersPerSecondToVoltage(optimizedState.speedMetersPerSecond);


        this.setDesiredModuleDriveVoltage(driveVoltage);
        this.swerveModuleIO.setDesiredModuleAngle(optimizedState.angle);

    }

    public void setDesiredModuleDriveVoltage(double driveVoltage) {
        /*voltageColumn.add(swerveModuleIO.getAppliedDriveMotorVolts());
        velocityColumn.add(getDriveMotorSpeedInMetersPerSecond());
        LogIOInputs.logToStateTable(voltageColumn, "VoltageToVelocityTeleop/" + swerveModuleName + "/Voltage");
        LogIOInputs.logToStateTable(velocityColumn, "VoltageToVelocityTeleop/" + swerveModuleName + "/Velocity");*/
        this.swerveModuleIO.setDesiredModuleDriveVoltage(driveVoltage);
    }

    public void resetTurningMotorToAbsolute() {
        swerveModuleIO.resetTurningMotorToAbsolute();
    }

    public Rotation2d getAbsoluteEncoderPosition() {
        return Rotation2d.fromRotations(swerveModuleIO.getOutputs().turnMotorAbsolutePositionRotations);
    }
    public Rotation2d getRelativeEncoderPosition() {
        return Rotation2d.fromRotations(swerveModuleIO.getOutputs().turnMotorRelativePositionRotations);
    }

    DesiredMetersPerSecondToVoltageLerpTable lerpTable = new DesiredMetersPerSecondToVoltageLerpTable();
   
    public double metersPerSecondToVoltage(double desiredMetersPerSecond) {

        if (desiredMetersPerSecond < SwerveDriveConstants.mpsLerpTableCutoff) {
            double percentOfMaxSpeed = desiredMetersPerSecond / SwerveDriveConstants.maxSpeedMPS;
            double appliedFeedforwardVoltage = (desiredMetersPerSecond != 0 ? SwerveDriveConstants.voltageFeedForward : 0) *
                Math.signum(desiredMetersPerSecond);
            double unclampedVoltage = percentOfMaxSpeed *
                SwerveDriveConstants.voltageForMaxSpeed +
                appliedFeedforwardVoltage;
            double clampedVoltage = MathUtil.clamp(unclampedVoltage, -SwerveDriveConstants.voltageForMaxSpeed, SwerveDriveConstants.voltageForMaxSpeed);
            return clampedVoltage;
        } else {
            return lerpTable.metersPerSecondToVoltage(desiredMetersPerSecond, desiredMetersPerSecond-1);
        }
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
            swerveModuleIO.getOutputs().driveMotorDistanceMeters, Rotation2d.fromRotations(swerveModuleIO.getOutputs().turnMotorRelativePositionRotations));
    }

    public double getModuleDriveDistance() {
        return swerveModuleIO.getOutputs().driveMotorDistanceMeters * SwerveDriveConstants.driveConversionPositionFactor;
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
        return this.swerveModuleIO.getOutputs().driveMotorRPS * 60 * SwerveDriveConstants.driveConversionVelocityFactor;
    }

    /**
     * The current angle of the module's wheel
     * @return Rotation2d: The module's wheel angle
     */
    private Rotation2d getWheelRotationAsRotation2d() {
        return Rotation2d.fromRotations(swerveModuleIO.getOutputs().turnMotorRelativePositionRotations);
    }
}
