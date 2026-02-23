package binarycrows.robot.SeasonCode.SubStateManagers.Hood;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.HoodConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import binarycrows.robot.Utils.Tuning.RuntimeTunablePIDValues;
import edu.wpi.first.math.geometry.Rotation2d;

public class HoodTalonFXSIntegratedPID implements HoodIO {
    public HoodOutputs outputs;

    private Rotation2d targetPosition = Rotation2d.kZero;

    private TalonFXS hoodMotor;
    private CANcoder hoodEncoder;

    private Rotation2d hoodEncoderOffset;

    private VoltageOut hoodVoltageRequest = new VoltageOut(0);

    private PositionDutyCycle hoodControlRequest = new PositionDutyCycle(0);

    private RuntimeTunablePIDValues hoodPIDConstantTuner;
    
    public HoodTalonFXSIntegratedPID(HoodOutputs outputs) {
        this.outputs = outputs;
        hoodMotor = new TalonFXS(CANIDs.RIO.hoodMotor);

        TalonFXSConfiguration motorConfig = new TalonFXSConfiguration();
        motorConfig.MotorOutput.Inverted = HoodConstants.motorInverted;
        motorConfig.MotorOutput.NeutralMode = HoodConstants.motorNeutralMode;

        this.hoodMotor.getVelocity().setUpdateFrequency(20);
        this.hoodMotor.getAcceleration().setUpdateFrequency(20);
        this.hoodMotor.getPosition().setUpdateFrequency(20);
        this.hoodMotor.getTorqueCurrent().setUpdateFrequency(50);
        
        motorConfig.Voltage.PeakForwardVoltage = HoodConstants.maximumVoltage;
        motorConfig.Voltage.PeakReverseVoltage = -HoodConstants.maximumVoltage;

        motorConfig.CurrentLimits.StatorCurrentLimit = HoodConstants.statorCurrentLimit;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        this.hoodMotor.getConfigurator().apply(motorConfig);

        hoodEncoder = new CANcoder(CANIDs.RIO.hoodEncoder);
        CANcoderConfiguration hoodEncoderConfig = new CANcoderConfiguration();
        MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs();
        magnetConfigs.AbsoluteSensorDiscontinuityPoint = 1;
        magnetConfigs.MagnetOffset = 0.0;
        magnetConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        hoodEncoderConfig.MagnetSensor = magnetConfigs;
        hoodEncoder.getConfigurator().apply(hoodEncoderConfig);

        hoodEncoderOffset = HoodConstants.hoodEncoderOffset;

        configurePID();

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        resetMotorToAbsolute();

    }

    @Override
    public void update() {

        outputs.motorVelocityRPS = hoodMotor.getVelocity().getValueAsDouble();
        outputs.motorAppliedVoltage = hoodMotor.getMotorVoltage().getValueAsDouble();;
        outputs.motorSupplyAmps = hoodMotor.getSupplyCurrent().getValueAsDouble();
        outputs.motorTorqueAmps = hoodMotor.getTorqueCurrent().getValueAsDouble();

        outputs.motorRotation = Rotation2d.fromRotations(hoodMotor.getRotorPosition().getValueAsDouble());
        outputs.encoderRotation = Rotation2d.fromRotations(hoodEncoder.getAbsolutePosition().getValueAsDouble());
        outputs.hoodRotation = Rotation2d.fromRotations(hoodEncoder.getAbsolutePosition().getValueAsDouble()).minus(hoodEncoderOffset);
        outputs.hoodRotationalVelocityRadPerSec = hoodEncoder.getVelocity().getValueAsDouble() * 2 * Math.PI;
        outputs.targetPosition = targetPosition;
        outputs.distanceFromSetpoint = outputs.targetPosition.minus(outputs.hoodRotation);

        updatePIDValuesFromNetworkTables();
    }

    private double getGravityFF() {
        return this.outputs.encoderRotation.getSin() * HoodConstants.hoodGravityFF;
    }

    public void updatePIDValuesFromNetworkTables() {


        if (MetaConstants.inProduction) return; // Don't run at competitions!
        if(this.hoodPIDConstantTuner.hasAnyPIDValueChanged()) {
            this.hoodMotor.getConfigurator().apply(hoodPIDConstantTuner.generatePIDFFConfigs());
        }

    }

    private void configurePID() {
        Slot0Configs turretPIDConfig = new Slot0Configs();

        turretPIDConfig.kP = HoodConstants.hoodPIDValueP;
        turretPIDConfig.kI = HoodConstants.hoodPIDValueI;
        turretPIDConfig.kD = HoodConstants.hoodPIDValueD;
        turretPIDConfig.kS = HoodConstants.hoodPIDValueFF;

        this.hoodMotor.getConfigurator().apply(turretPIDConfig);

        hoodPIDConstantTuner = new RuntimeTunablePIDValues("Hood/PIDValues",
        HoodConstants.hoodPIDValueP, HoodConstants.hoodPIDValueI, HoodConstants.hoodPIDValueD, HoodConstants.hoodPIDValueFF);

    }

    @Override
    public void setRotorVoltage(double voltage) {
        hoodVoltageRequest = new VoltageOut(voltage);
        hoodMotor.setControl(hoodVoltageRequest);
    }

    @Override
    public void setTargetPosition(Rotation2d position) {
        targetPosition = position;
        hoodControlRequest = new PositionDutyCycle(targetPosition.getRotations());
        hoodControlRequest.FeedForward = getGravityFF();
        hoodMotor.setControl(hoodControlRequest);
    }

    @Override
    public void resetMotorToAbsolute() {
        this.hoodMotor.setPosition((
            this.hoodEncoder.getAbsolutePosition().getValueAsDouble() 
            - this.hoodEncoderOffset.getRotations()) * SwerveDriveConstants.turnGearRatio);
    }

    @Override
    public HoodOutputs getOutputs() {
        return this.outputs;
    }
}
