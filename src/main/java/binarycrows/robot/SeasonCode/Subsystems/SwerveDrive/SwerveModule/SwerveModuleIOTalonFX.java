package binarycrows.robot.SeasonCode.Subsystems.SwerveDrive.SwerveModule;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import binarycrows.robot.SeasonCode.Constants.SwerveModuleConstants;
import binarycrows.robot.Utils.Tuning.RuntimeTunablePIDValues;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class SwerveModuleIOTalonFX implements SwerveModuleIO{

    private String swerveModuleName; 

    private TalonFX driveMotor;
    private TalonFX turnMotor;

    private CANcoder turnAbsoluteEncoder;

    private double turningAbsoluteEncoderOffset;

    private VelocityVoltage driveControlRequest = new VelocityVoltage(0);
    // If this works and we have a CTRE subscripting try enablign FOC
    private VoltageOut driveControlVoltageRequest = new VoltageOut(0);

    private PositionDutyCycle turnControlRequest = new PositionDutyCycle(0);

    private RuntimeTunablePIDValues driveMotorPIDConstantTuner;
    private RuntimeTunablePIDValues turnMotorPIDConstantTuner;

    /**
     * Creates a SwerveModuleIOTalonFX object and completes all configuration for the module
     * @param swerveModuleName String: The name of the module. List of valid module names can be found in SwerveDriveConstants
     */
    public SwerveModuleIOTalonFX(String swerveModuleName) {
        this.swerveModuleName = swerveModuleName;

        switch (this.swerveModuleName) {
            case SwerveDriveConstants.kFrontLeftModuleName:
                configDriveMotor(SwerveDriveConstants.kFrontLeftDriveMotorCANID, SwerveDriveConstants.kFrontLeftDriveMotorInverted);
                configTurnMotor(SwerveDriveConstants.kFrontLeftTurnMotorCANID, SwerveDriveConstants.kFrontLeftTurnMotorInverted);
                configTurningAbsoluteEncoder(SwerveDriveConstants.kFrontLeftTurningAbsoluteEncoderCANID, SwerveDriveConstants.kFrontLeftTurningAbsoluteEncoderOffsetRotations);
                break;
            case SwerveDriveConstants.kFrontRightModuleName:
                configDriveMotor(SwerveDriveConstants.kFrontRightDriveMotorCANID, SwerveDriveConstants.kFrontRightDriveMotorInverted);
                configTurnMotor(SwerveDriveConstants.kFrontRightTurnMotorCANID, SwerveDriveConstants.kFrontRightTurnMotorInverted);
                configTurningAbsoluteEncoder(SwerveDriveConstants.kFrontRightTurningAbsoluteEncoderCANID, SwerveDriveConstants.kFrontRightTurningAbsoluteEncoderOffsetRotations);
                break;
            case SwerveDriveConstants.kBackLeftModuleName:
                configDriveMotor(SwerveDriveConstants.kBackLeftDriveMotorCANID, SwerveDriveConstants.kBackLeftDriveMotorInverted);
                configTurnMotor(SwerveDriveConstants.kBackLeftTurnMotorCANID, SwerveDriveConstants.kBackLeftTurnMotorInverted);
                configTurningAbsoluteEncoder(SwerveDriveConstants.kBackLeftTurningAbsoluteEncoderCANID, SwerveDriveConstants.kBackLeftTurningAbsoluteEncoderOffsetRotations);
                break;
            case SwerveDriveConstants.kBackRightModuleName:
                configDriveMotor(SwerveDriveConstants.kBackRightDriveMotorCANID, SwerveDriveConstants.kBackRightDriveMotorInverted);
                configTurnMotor(SwerveDriveConstants.kBackRightTurnMotorCANID, SwerveDriveConstants.kBackRightTurnMotorInverted);
                configTurningAbsoluteEncoder(SwerveDriveConstants.kBackRightTurningAbsoluteEncoderCANID, SwerveDriveConstants.kBackRightTurningAbsoluteEncoderOffsetRotations);
                break;
            default:
                throw new RuntimeException("Invalid Module Name: " + swerveModuleName + ", Please change to a valid name. List of valid names can be found in SwerveModuleConstants");
        }

        configDirvePID();
        configTurnPID();
        Timer.delay(.1); // We should see if we can reduce this to dramaticly increase robot boot time. 
        resetTurningMotorToAbsolute();

        addInitLogs();
    }

    /**
     * Configures the drive motor.
     * @param driveMotorID Integer: The CANID for the drive motor
     * @param driveMotorInverted Boolean: Wether or not the drive motor is inverted
     */
    private void configDriveMotor(int driveMotorID, boolean driveMotorInverted) {
        this.driveMotor = new TalonFX(driveMotorID, SwerveDriveConstants.kCANLoopName);
        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.MotorOutput.Inverted = driveMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // May need to set more fields becuase optimize useage 
        this.driveMotor.getVelocity().setUpdateFrequency(20);
        this.driveMotor.getAcceleration().setUpdateFrequency(20);
        this.driveMotor.getPosition().setUpdateFrequency(20);
        this.driveMotor.getTorqueCurrent().setUpdateFrequency(50);
        
    
        driveMotorConfig.Voltage.PeakForwardVoltage = SwerveModuleConstants.kDriveMotorMaxVoltageSparkMaxTalonFX;
        driveMotorConfig.Voltage.PeakReverseVoltage = -SwerveModuleConstants.kDriveMotorMaxVoltageSparkMaxTalonFX;
        //this.driveMotor.optimizeBusUtilization();
        this.driveMotor.getConfigurator().apply(driveMotorConfig);
    }

     /**
      * Configures the drive motor PID Controller. 
      */
    private void configDirvePID() {
        Slot0Configs drivePIDConfig = new Slot0Configs();

        drivePIDConfig.kP = SwerveModuleConstants.kPModuleDrivePIDValue;
        drivePIDConfig.kI = SwerveModuleConstants.kIModuleDrivePIDValue;
        drivePIDConfig.kD = SwerveModuleConstants.kDModuleDrivePIDValue;
        drivePIDConfig.kS = SwerveModuleConstants.kFFModuleDrivePIDValue;  

        this.driveMotor.getConfigurator().apply(drivePIDConfig);

        this.driveMotorPIDConstantTuner = new RuntimeTunablePIDValues("SwerveModule/DrivePIDValues",
            SwerveModuleConstants.kPModuleDrivePIDValue,
            SwerveModuleConstants.kIModuleDrivePIDValue,
            SwerveModuleConstants.kDModuleDrivePIDValue, 0);
    }

    /**
     * Configures the turn motor.
     * @param turnMotorID Integer: The CANID for the turn motor
     * @param turnMotorInverted Boolean: Wether or not the turn motor is inverted
     */
    private void configTurnMotor(int turnMotorID, boolean turnMotorInverted) {
        this.turnMotor = new TalonFX(turnMotorID, SwerveDriveConstants.kCANLoopName);
        TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();
        turnMotorConfig.MotorOutput.Inverted = turnMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        turnMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // May need to set more fields becuase optimize useage 
        this.turnMotor.getVelocity().setUpdateFrequency(20);
        this.turnMotor.getAcceleration().setUpdateFrequency(20);
        this.turnMotor.getPosition().setUpdateFrequency(20);
        this.turnMotor.getTorqueCurrent().setUpdateFrequency(50);

        turnMotorConfig.Voltage.PeakForwardVoltage = SwerveModuleConstants.kTurnMotorMaxVoltageSparkMaxTalonFX;
        turnMotorConfig.Voltage.PeakReverseVoltage = -SwerveModuleConstants.kTurnMotorMaxVoltageSparkMaxTalonFX;

        //this.turnMotor.optimizeBusUtilization();
        this.turnMotor.getConfigurator().apply(turnMotorConfig);
    }
    
    /**
      * Configures the turn motor PID Controller. 
     */
    private void configTurnPID() {
        Slot0Configs turnPIDConfig = new Slot0Configs();

        turnPIDConfig.kP = SwerveModuleConstants.kPModuleTurnPIDValue;
        turnPIDConfig.kI = SwerveModuleConstants.kIModuleTurnPIDValue;
        turnPIDConfig.kD = SwerveModuleConstants.kDModuleTurnPIDValue;
        turnPIDConfig.kS = SwerveModuleConstants.kFFModuleTurnPIDValue;  

        this.turnMotor.getConfigurator().apply(turnPIDConfig);

        this.turnMotorPIDConstantTuner = new RuntimeTunablePIDValues("SwerveModule/TurnPIDValues",
            SwerveModuleConstants.kPModuleTurnPIDValue,
            SwerveModuleConstants.kIModuleTurnPIDValue,
            SwerveModuleConstants.kDModuleTurnPIDValue, 0);
    }

    /**
     * Configures the turnAbsoluteEncoder
     * @param turnAbsoluteEncoderID Integer: The CANID of the turnAbsoluteEncoder
     * @param turningAbsoluteEncoderOffset Double: The offset of the turning absolute encoder. i.e the difforence between the encoders value and whare the wheel is straight.
     * Should be calucluted by useing a straight edge to make all wheels perfectly straight forward(Make sure all wheels are in the same orientation). 
     * From there the value can be gotton from Pheonix Tuner. 
     */
    private void configTurningAbsoluteEncoder(int turnAbsoluteEncoderID, double turningAbsoluteEncoderOffset) {
        this.turnAbsoluteEncoder = new CANcoder(turnAbsoluteEncoderID, SwerveDriveConstants.kCANLoopName);
        CANcoderConfiguration turningAbsoluteEncoderConfig = new CANcoderConfiguration();
        MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs();
        magnetConfigs.AbsoluteSensorDiscontinuityPoint = 1;
        magnetConfigs.MagnetOffset = 0.0f;
        magnetConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        turningAbsoluteEncoderConfig.MagnetSensor = magnetConfigs;
        this.turnAbsoluteEncoder.getConfigurator().apply(turningAbsoluteEncoderConfig);

        this.turningAbsoluteEncoderOffset = turningAbsoluteEncoderOffset;
    }

    /**
     * Resets the relitive built in encoder of the turn motor to be equal to the offsetted absolute angle reading from the absolute encoder.
     */
    public void resetTurningMotorToAbsolute() {
        this.turnMotor.setPosition((
            this.turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() 
            - this.turningAbsoluteEncoderOffset) * SwerveModuleConstants.kTurningGearRatio);
      }
    
    /**
     * Adds Swerve Module logs that never need to be updated to the dashboard.
     */
    private void addInitLogs() {
        Logger.recordOutput(SwerveModuleConstants.kSwerveModuleOutputLoggerBase + swerveModuleName + "TurningAbsoluteEncoderOffset", this.turningAbsoluteEncoderOffset);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.driveMotorRotations = this.driveMotor.getPosition().getValueAsDouble();
        inputs.driveMotorRPM = this.driveMotor.getVelocity().getValueAsDouble() * 60; // 60 For number of seconds in a minute.
        // This is done because TalonFX uses RPS instead of RMP
        inputs.driveMotorSpeedMetersPerSecond = inputs.driveMotorRPM * SwerveModuleConstants.kDriveConversionVelocityFactor;
        inputs.driveMotorDistanceMeters = (inputs.driveMotorRotations / SwerveModuleConstants.kDriveGearRatio) * SwerveModuleConstants.kWheelDistancePerRotation;
        inputs.driveMotorAppliedVolts = this.driveMotor.getMotorVoltage().getValueAsDouble();
        inputs.driveMotorCurrentAmps = new double[] {this.driveMotor.getSupplyCurrent().getValueAsDouble()};

        inputs.turnMotorAbsolutePositionRotations = this.turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
        inputs.turnMotorRelitivePositionRotations = this.turnMotor.getPosition().getValueAsDouble();
        inputs.wheelAngleRelitivePositionRotations = inputs.turnMotorRelitivePositionRotations / SwerveModuleConstants.kTurningGearRatio;
        inputs.turnMotorRPM = this.turnMotor.getVelocity().getValueAsDouble() * 60; // 60 For number of seconds in a minute.
        // This is done because TalonFX uses RPS instead of RMP
        inputs.turnMotorAppliedVolts = this.turnMotor.getMotorVoltage().getValueAsDouble();
        inputs.turnMotorCurrentAmps = new double[] {this.turnMotor.getSupplyCurrent().getValueAsDouble()};
        
        updatePIDValuesFromNetworkTables();
    }

    /**
     * WORNING!!! There should only be one call of this method and that
     *  call should be commented out before going to a competition. 
     * Updates the PID values for the module bassed on network tables.
     * Must be called periodicly.
     */
    private void updatePIDValuesFromNetworkTables() {
        double[] currentDrivePIDValues = this.driveMotorPIDConstantTuner.getUpdatedPIDConstants();
        if(this.driveMotorPIDConstantTuner.hasAnyPIDValueChanged()) {
            Slot0Configs newDrivePIDConfigs = new Slot0Configs();
            newDrivePIDConfigs.kP = currentDrivePIDValues[0];
            newDrivePIDConfigs.kI = currentDrivePIDValues[1];
            newDrivePIDConfigs.kD = currentDrivePIDValues[2];
            newDrivePIDConfigs.kS = currentDrivePIDValues[3];
            this.driveMotor.getConfigurator().apply(newDrivePIDConfigs);
        }
        
        double[] currentTurnPIDValues = this.turnMotorPIDConstantTuner.getUpdatedPIDConstants();
        if(this.turnMotorPIDConstantTuner.hasAnyPIDValueChanged()) {
            Slot0Configs newTurnPIDConfigs = new Slot0Configs();
            newTurnPIDConfigs.kP = currentTurnPIDValues[0];
            newTurnPIDConfigs.kI = currentTurnPIDValues[1];
            newTurnPIDConfigs.kD = currentTurnPIDValues[2];
            newTurnPIDConfigs.kS = currentTurnPIDValues[3];
            this.turnMotor.getConfigurator().apply(newTurnPIDConfigs);
        }
    }

    @Override
    public void setDesiredModuleVelocityRPM(double desiredRPM) {
        Logger.recordOutput(SwerveModuleConstants.kSwerveModuleOutputLoggerBase + swerveModuleName + "DesiredRPM", desiredRPM);
        this.driveControlRequest.Velocity = desiredRPM / 60; // 60 For number of seconds in a minute.
        // This is done because TalonFX uses RPS instead of RMP
        this.driveMotor.setControl(this.driveControlRequest);
    }

    @Override
    public void setDesiredModuleDriveVoltage(double desiredVoltage) {
        this.driveControlVoltageRequest.Output = desiredVoltage;
        this.driveMotor.setControl(this.driveControlVoltageRequest);
    }

    @Override
    public void setDesiredModuleAngle(Rotation2d desiredModuleAngle) {
        double desiredModuleRotations = desiredModuleAngle.getRotations(); 
        double desiredMotorRotation = desiredModuleRotations * SwerveModuleConstants.kTurningGearRatio;

        Logger.recordOutput(SwerveModuleConstants.kSwerveModuleOutputLoggerBase + swerveModuleName + "DesiredWheelAngleRelitivePositionRotations" , desiredModuleRotations);
        Logger.recordOutput(SwerveModuleConstants.kSwerveModuleOutputLoggerBase + swerveModuleName + "DesiredTurnMotorRelitivePositionRotations" , desiredMotorRotation);

        this.turnControlRequest.Position = desiredMotorRotation;
        this.turnMotor.setControl(this.turnControlRequest);
    }
}