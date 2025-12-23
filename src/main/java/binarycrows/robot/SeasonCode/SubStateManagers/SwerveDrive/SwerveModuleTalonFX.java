package binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive;

import java.util.HashMap;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import binarycrows.robot.Utils.LogIOInputs;
import binarycrows.robot.Utils.Tuning.RuntimeTunablePIDValues;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class SwerveModuleTalonFX implements SwerveModuleIO {

    public String swerveModuleName; 

    public SwerveModuleOutputs outputs;

    private TalonFX driveMotor;
    private TalonFX turnMotor;

    private CANcoder turnAbsoluteEncoder;

    public double turningAbsoluteEncoderOffset;

    // If this works and we have a CTRE subscripting try enablign FOC
    private VoltageOut driveControlVoltageRequest = new VoltageOut(0);
    
    private PositionDutyCycle turnControlRequest = new PositionDutyCycle(0);

    private RuntimeTunablePIDValues turnMotorPIDConstantTuner;

    
    public double driveMotorRotations = 0.0;
    public double driveMotorSpeedMetersPerSecond = 0.0;
    public double driveMotorDistanceMeters = 0.0;
    public double driveMotorAppliedVolts = 0.0;
    public double[] driveMotorCurrentAmps = new double[] {};

    public double turnMotorAbsolutePositionRotations = 0.0;
    public double turnMotorRelitivePositionRotations = 0.0;
    public double turnMotorRPM = 0.0;
    public double turnMotorAppliedVolts = 0.0;
    public double[] turnMotorCurrentAmps = new double[] {};


    /**
     * Creates a SwerveModuleIOTalonFX object and completes all configuration for the module
     * @param swerveModuleName String: The name of the module. List of valid module names can be found in SwerveDriveConstants
     */
    public SwerveModuleTalonFX(String swerveModuleName, SwerveModuleOutputs outputs) {
        this.swerveModuleName = swerveModuleName;
        this.outputs = outputs;

        switch (this.swerveModuleName) {
            case SwerveDriveConstants.frontLeftModuleName:
                configDriveMotor(CANIDs.frontLeftDriveMotor, SwerveDriveConstants.frontLeftDriveInverted);

                configTurnMotor(CANIDs.frontLeftTurnMotor, SwerveDriveConstants.frontLeftTurnInverted);
                configTurningAbsoluteEncoder(CANIDs.frontLeftTurnEncoderCANID, SwerveDriveConstants.frontLeftTurnEncoderOffset);

                break;
            case SwerveDriveConstants.frontRightModuleName:
                configDriveMotor(CANIDs.frontRightDriveMotor, SwerveDriveConstants.frontRightDriveInverted);

                configTurnMotor(CANIDs.frontRightTurnMotor, SwerveDriveConstants.frontRightTurnInverted);
                configTurningAbsoluteEncoder(CANIDs.frontRightTurnEncoderCANID, SwerveDriveConstants.frontRightTurnEncoderOffset);

                break;
            case SwerveDriveConstants.backLeftModuleName:
                configDriveMotor(CANIDs.backLeftDriveMotor, SwerveDriveConstants.backLeftDriveInverted);

                configTurnMotor(CANIDs.backLeftTurnMotor, SwerveDriveConstants.backLeftTurnInverted);
                configTurningAbsoluteEncoder(CANIDs.backLeftTurnEncoderCANID, SwerveDriveConstants.backLeftTurnEncoderOffset);

                break;
            case SwerveDriveConstants.backRightModuleName:
                configDriveMotor(CANIDs.backRightDriveMotor, SwerveDriveConstants.backRightTurnInverted);

                configTurnMotor(CANIDs.backRightTurnMotor, SwerveDriveConstants.backRightTurnInverted);
                configTurningAbsoluteEncoder(CANIDs.backRightTurnEncoderCANID, SwerveDriveConstants.backRightTurnEncoderOffset);

                break;
            default:
                throw new RuntimeException("The undefined module " + swerveModuleName + " was used. Please change to a valid name found in the SwerveDriveConstants.java file.");
        }
        configureTurnPID();
        //Timer.delay(.1); // We should see if we can reduce this to dramaticly increase robot boot time. 
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        resetTurningMotorToAbsolute();
    }

    /**
     * Configures the drive motor.
     * @param driveMotorID Integer: The CANID for the drive motor
     * @param driveMotorInverted Boolean: Wether or not the drive motor is inverted
     */
    private void configDriveMotor(int driveMotorID, boolean driveMotorInverted) {
        this.driveMotor = new TalonFX(driveMotorID, SwerveDriveConstants.CANLoopName);
        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.MotorOutput.Inverted = driveMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        this.driveMotor.getVelocity().setUpdateFrequency(20);
        this.driveMotor.getAcceleration().setUpdateFrequency(20);
        this.driveMotor.getPosition().setUpdateFrequency(20);
        this.driveMotor.getTorqueCurrent().setUpdateFrequency(50);
        
    
        driveMotorConfig.Voltage.PeakForwardVoltage = SwerveDriveConstants.maxDriveMotorVoltage;
        driveMotorConfig.Voltage.PeakReverseVoltage = -SwerveDriveConstants.maxDriveMotorVoltage;


        
        //this.driveMotor.optimizeBusUtilization();
        this.driveMotor.getConfigurator().apply(driveMotorConfig);
        this.driveMotor.setPosition(0);
        
    }

    /**
     * Configures the turn motor.
     * @param turnMotorID Integer: The CANID for the turn motor
     * @param turnMotorInverted Boolean: Wether or not the turn motor is inverted
     */
    private void configTurnMotor(int turnMotorID, boolean turnMotorInverted) {
        this.turnMotor = new TalonFX(turnMotorID, SwerveDriveConstants.CANLoopName);
        TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();
        turnMotorConfig.MotorOutput.Inverted = turnMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        turnMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // May need to set more fields becuase optimize useage 
        this.turnMotor.getVelocity().setUpdateFrequency(20);
        this.turnMotor.getAcceleration().setUpdateFrequency(20);
        this.turnMotor.getPosition().setUpdateFrequency(20);
        this.turnMotor.getTorqueCurrent().setUpdateFrequency(50);

        turnMotorConfig.Voltage.PeakForwardVoltage = SwerveDriveConstants.maxTurnMotorVoltage;
        turnMotorConfig.Voltage.PeakReverseVoltage = -SwerveDriveConstants.maxTurnMotorVoltage;


        //this.turnMotor.optimizeBusUtilization();
        this.turnMotor.getConfigurator().apply(turnMotorConfig);
        this.driveMotor.setPosition(0);

    }

    // Drive motor getters
    public double getDriveMotorRPS() {
        return this.driveMotor.getRotorVelocity().getValueAsDouble();
    }
    public double getDriveMotorMPS() {
        return getDriveMotorRPS() * 60 * SwerveDriveConstants.driveConversionVelocityFactor;
    }
    public double getDriveMotorDistanceRotations() {
        return this.driveMotor.getPosition().getValueAsDouble();
    }
    public double getDriveMotorDistanceMeters() {
        return getDriveMotorDistanceRotations() * SwerveDriveConstants.driveConversionPositionFactor;
    }
    public Rotation2d getAbsoluteEncoderPosition() {
        return Rotation2d.fromRotations(this.turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() - turningAbsoluteEncoderOffset);
    }
    public double getAppliedDriveMotorVolts() {
        return this.driveMotor.getMotorVoltage().getValueAsDouble();
    }
    public double getDriveMotorSupplyAmps() {
        return this.driveMotor.getSupplyCurrent().getValueAsDouble();
    }
    public double getDriveMotorTorqueAmps() {
        return this.driveMotor.getTorqueCurrent().getValueAsDouble();
    }

    // Turn motor getters
    public double getTurnMotorRPS() {
        return this.turnMotor.getRotorVelocity().getValueAsDouble();
    }
    public double getTurnMotorMPS() {
        return getTurnMotorRPS() * 60 * SwerveDriveConstants.driveConversionVelocityFactor;
    }
    public double getTurnMotorAbsolutePositionRotations() {
        return this.turnAbsoluteEncoder.getPosition().getValueAsDouble();
    }
    public double getTurnMotorRelativePositionRotations() {
        return this.turnMotor.getPosition().getValueAsDouble() / SwerveDriveConstants.turnGearRatio;
    }
    public double getAppliedTurnMotorVolts() {
        return this.turnMotor.getMotorVoltage().getValueAsDouble();
    }
    public double getTurnMotorSupplyAmps() {
        return this.turnMotor.getSupplyCurrent().getValueAsDouble();
    }
    public double getTurnMotorTorqueAmps() {
        return this.turnMotor.getTorqueCurrent().getValueAsDouble();
    }

    /**
      * Configures the turn motor PID Controller. 
     */
    private void configureTurnPID() {
        Slot0Configs turnPIDConfig = new Slot0Configs();

        turnPIDConfig.kP = SwerveDriveConstants.turnPIDValueP;
        turnPIDConfig.kI = SwerveDriveConstants.turnPIDValueI;
        turnPIDConfig.kD = SwerveDriveConstants.turnPIDValueD;
        turnPIDConfig.kS = SwerveDriveConstants.turnPIDValueFF;  

        this.turnMotor.getConfigurator().apply(turnPIDConfig);

        this.turnMotorPIDConstantTuner = new RuntimeTunablePIDValues("SwerveModule/" + swerveModuleName + "/TurnPIDValues",
            SwerveDriveConstants.turnPIDValueP,
            SwerveDriveConstants.turnPIDValueI,
            SwerveDriveConstants.turnPIDValueD, 
            SwerveDriveConstants.turnPIDValueFF);
    }

    /**
     * Configures the turnAbsoluteEncoder
     * @param turnAbsoluteEncoderID Integer: The CANID of the turnAbsoluteEncoder
     * @param turningAbsoluteEncoderOffset Double: The offset of the turning absolute encoder. i.e the difforence between the encoders value and whare the wheel is straight.
     * Should be calucluted by useing a straight edge to make all wheels perfectly straight forward(Make sure all wheels are in the same orientation). 
     * From there the value can be gotton from Pheonix Tuner. 
     */
    private void configTurningAbsoluteEncoder(int turnAbsoluteEncoderID, double turningAbsoluteEncoderOffset) {
        this.turnAbsoluteEncoder = new CANcoder(turnAbsoluteEncoderID, SwerveDriveConstants.CANLoopName);
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
     * There should only be one call of this method and that
     *  call should not exist at a competition. 
     * Updates the PID values for the module bassed on network tables.
     * Must be called periodically.
     */
    public void updatePIDValuesFromNetworkTables() {


        if (MetaConstants.inProduction) return; // Don't run at competitions!
        double[] currentTurnPIDValues = this.turnMotorPIDConstantTuner.getUpdatedPIDConstants();
        if(this.turnMotorPIDConstantTuner.hasAnyPIDValueChanged()) {
            Slot0Configs newTurnPIDConfigs = new Slot0Configs();
            newTurnPIDConfigs.kP = currentTurnPIDValues[0];
            newTurnPIDConfigs.kI = currentTurnPIDValues[1];
            newTurnPIDConfigs.kD = currentTurnPIDValues[2];
            newTurnPIDConfigs.kS = currentTurnPIDValues[3];
            this.turnMotor.getConfigurator().apply(newTurnPIDConfigs);
        }

        LogIOInputs.logToStateTable(this.turnControlRequest.Position, swerveModuleName + "/DesiredRotation");
        LogIOInputs.logToStateTable(this.turnMotor.getPosition().getValueAsDouble(), swerveModuleName + "/ActualRotation");

    }

    @Override
    public void setDesiredModuleDriveVoltage(double desiredVoltage) {


        this.driveControlVoltageRequest.Output = desiredVoltage;
        this.driveMotor.setControl(this.driveControlVoltageRequest);


    }

    @Override
    public void setDesiredModuleAngle(Rotation2d desiredModuleAngle) {
        double desiredModuleRotations = desiredModuleAngle.getRotations(); 
        double desiredMotorRotation = desiredModuleRotations * SwerveDriveConstants.turnGearRatio;

        this.turnControlRequest.Position = desiredMotorRotation;
        this.turnMotor.setControl(this.turnControlRequest);
    }

    @Override
    public void update() {
        outputs.driveMotorAppliedVolts = getAppliedDriveMotorVolts();
        outputs.driveMotorSupplyAmps = getDriveMotorSupplyAmps();
        outputs.driveMotorTorqueAmps = getDriveMotorTorqueAmps();
        outputs.driveMotorDistanceRotations = getDriveMotorDistanceRotations();
        outputs.driveMotorDistanceMeters = getDriveMotorDistanceMeters();
        outputs.driveMotorRPS = getDriveMotorRPS();
        outputs.driveMotorSpeedMetersPerSecond = getDriveMotorMPS();

        outputs.turnMotorAppliedVolts = getAppliedTurnMotorVolts();
        outputs.turnMotorSupplyAmps = getTurnMotorSupplyAmps();
        outputs.turnMotorTorqueAmps = getTurnMotorTorqueAmps();
        outputs.turnMotorAbsolutePositionRotations = getTurnMotorAbsolutePositionRotations();
        outputs.turnMotorRelativePositionRotations = getTurnMotorRelativePositionRotations();
        outputs.turnMotorRPS = getTurnMotorRPS();
        outputs.turnMotorDesiredPositionRotations = this.turnControlRequest.Position;


        updatePIDValuesFromNetworkTables();
    }
    
    @Override
    public SwerveModuleOutputs getOutputs() {
        return outputs;
    }

    /**
     * Resets the relitive built in encoder of the turn motor to be equal to the offsetted absolute angle reading from the absolute encoder.
     */
    @Override
    public void resetTurningMotorToAbsolute() {
        this.turnMotor.setPosition((
            this.turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() 
            - this.turningAbsoluteEncoderOffset) * SwerveDriveConstants.turnGearRatio);
    }
}
