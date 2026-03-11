package binarycrows.robot.SeasonCode.SubStateManagers.Turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.TurretConstants;
import binarycrows.robot.Utils.Tuning.RuntimeTunablePIDValues;
import edu.wpi.first.math.geometry.Rotation2d;

public class TurretTalonFX implements TurretIO {
    public TurretOutputs outputs;

    private Rotation2d targetPosition = Rotation2d.kZero;

    private TalonFX turretMotor;
    private CANcoder turretEncoder;


    private VoltageOut turretVoltageRequest = new VoltageOut(0);

    private PositionDutyCycle turretControlRequest = new PositionDutyCycle(0);

    private RuntimeTunablePIDValues turretPIDConstantTuner;
    
    public TurretTalonFX(TurretOutputs outputs) {
        this.outputs = outputs;
        turretMotor = new TalonFX(CANIDs.RIO.turretMotor);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = TurretConstants.motorInverted;
        motorConfig.MotorOutput.NeutralMode = TurretConstants.motorNeutralMode;

        this.turretMotor.getVelocity().setUpdateFrequency(20);
        this.turretMotor.getAcceleration().setUpdateFrequency(20);
        this.turretMotor.getPosition().setUpdateFrequency(20);
        this.turretMotor.getTorqueCurrent().setUpdateFrequency(50);
        
        motorConfig.Feedback.SensorToMechanismRatio = TurretConstants.motorToTurretGearRatio;

        motorConfig.Voltage.PeakForwardVoltage = TurretConstants.maximumVoltage; 
        motorConfig.Voltage.PeakReverseVoltage = -TurretConstants.maximumVoltage;
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = TurretConstants.torqueCurrentLimit;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -TurretConstants.torqueCurrentLimit;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 
            0.5 + Rotation2d.fromRadians(TurretConstants.forwardOverextensionRad).getRotations();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 
            -0.5 - Rotation2d.fromRadians(TurretConstants.reverseOverextensionRad).getRotations();
        
        this.turretMotor.getConfigurator().apply(motorConfig);

        turretEncoder = new CANcoder(CANIDs.RIO.turretEncoder);
        CANcoderConfiguration turretEncoderConfig = new CANcoderConfiguration();
        MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs();
        magnetConfigs.AbsoluteSensorDiscontinuityPoint = 1;
        magnetConfigs.MagnetOffset = 0.0;
        magnetConfigs.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        turretEncoderConfig.MagnetSensor = magnetConfigs;
        turretEncoder.getConfigurator().apply(turretEncoderConfig);


        configurePID();

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        update();
        resetMotorToAbsolute();

    }

    @Override
    public void update() {

        outputs.motorVelocityRPS = turretMotor.getVelocity().getValueAsDouble();
        outputs.motorAppliedVoltage = turretMotor.getMotorVoltage().getValueAsDouble();;
        outputs.motorSupplyAmps = turretMotor.getSupplyCurrent().getValueAsDouble();
        outputs.motorTorqueAmps = turretMotor.getTorqueCurrent().getValueAsDouble();

        outputs.motorRotation = Rotation2d.fromRotations(turretMotor.getPosition().getValueAsDouble());
        outputs.absoluteEncoderValueRotations = turretEncoder.getAbsolutePosition().getValueAsDouble();
        outputs.relativeEncoderValueRotations = turretMotor.getRotorPosition().getValueAsDouble() * TurretConstants.motorToTurretGearRatio;

        outputs.encoderRotation = getEncoderRotation();
        outputs.turretRotationalVelocityRadPerSec = turretEncoder.getVelocity().getValueAsDouble() * 2 * Math.PI;
        outputs.targetPosition = targetPosition;
        outputs.distanceFromSetpoint = outputs.targetPosition.minus(outputs.encoderRotation);

        if (outputs.encoderRotation != outputs.motorRotation && outputs.motorRotation.getRotations() <= 1 && outputs.motorRotation.getRotations() >= 0) {
            //resetMotorToAbsolute();
        }

        updatePIDValuesFromNetworkTables();
    }
    
    private Rotation2d getEncoderRotation() {
        double encoderValue = outputs.absoluteEncoderValueRotations;

        // Proportion (zero to one) that the encoder is at between min and max rotation
        double encoderProportion = 
        (encoderValue - TurretConstants.turretEncoderOffset.getRotations()) / 
        TurretConstants.encoderHalfCircleDistance.getRotations();
        
        // Value of encoder proportion within 
        double scaledValue = encoderProportion * Math.PI;

        return Rotation2d.fromRadians(scaledValue);
    }

    public void updatePIDValuesFromNetworkTables() {


        if (MetaConstants.inProduction) return; // Don't run at competitions!
        if(this.turretPIDConstantTuner.hasAnyPIDValueChanged()) {
            this.turretMotor.getConfigurator().apply(turretPIDConstantTuner.generatePIDFFConfigs());
        }

    }

    private void configurePID() {
        Slot0Configs turretPIDConfig = new Slot0Configs();

        turretPIDConfig.kP = TurretConstants.turretPIDValueP;
        turretPIDConfig.kI = TurretConstants.turretPIDValueI;
        turretPIDConfig.kD = TurretConstants.turretPIDValueD;
        turretPIDConfig.kS = TurretConstants.turretPIDValueFF;

        this.turretMotor.getConfigurator().apply(turretPIDConfig);

        turretPIDConstantTuner = new RuntimeTunablePIDValues("Turret/PIDValues",
        TurretConstants.turretPIDValueP, TurretConstants.turretPIDValueI, TurretConstants.turretPIDValueD, TurretConstants.turretPIDValueFF);

    }

    @Override
    public void setRotorVoltage(double voltage) {
        turretVoltageRequest = new VoltageOut(voltage);
        turretMotor.setControl(turretVoltageRequest);
    }

    @Override
    public void setTargetPosition(Rotation2d position) {

        targetPosition = position;
        turretControlRequest = new PositionDutyCycle(targetPosition.getRotations());
        turretMotor.setControl(turretControlRequest);
    }

    @Override
    public void resetMotorToAbsolute() {
        this.turretMotor.setPosition((
            outputs.encoderRotation.getRotations()));
    }

    public TurretOutputs getOutputs() {return outputs;};
}
