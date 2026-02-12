package binarycrows.robot.SeasonCode.SubStateManagers.Turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import binarycrows.robot.SeasonCode.Constants.TurretConstants;
import binarycrows.robot.Utils.ConversionUtils;
import binarycrows.robot.Utils.LogIOInputs;
import binarycrows.robot.Utils.Tuning.RuntimeTunablePIDValues;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOTalonFX implements TurretIO {
    public TurretOutputs outputs;

    private Rotation2d targetPosition = Rotation2d.kZero;

    private TalonFX turretMotor;
    private CANcoder turretEncoder;

    private Rotation2d turretEncoderOffset;

    private VoltageOut turretVoltageRequest = new VoltageOut(0);

    private PositionDutyCycle turretControlRequest = new PositionDutyCycle(0);

    private RuntimeTunablePIDValues turretPIDConstantTuner;
    
    public TurretIOTalonFX(TurretOutputs outputs) {
        this.outputs = outputs;
        turretMotor = new TalonFX(CANIDs.RIO.turretMotor);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = TurretConstants.motorInverted;
        motorConfig.MotorOutput.NeutralMode = TurretConstants.motorNeutralMode;

        this.turretMotor.getVelocity().setUpdateFrequency(20);
        this.turretMotor.getAcceleration().setUpdateFrequency(20);
        this.turretMotor.getPosition().setUpdateFrequency(20);
        this.turretMotor.getTorqueCurrent().setUpdateFrequency(50);

        motorConfig.Voltage.PeakForwardVoltage = TurretConstants.maximumVoltage;
        motorConfig.Voltage.PeakReverseVoltage = -TurretConstants.maximumVoltage;

        this.turretMotor.getConfigurator().apply(motorConfig);

        turretEncoder = new CANcoder(CANIDs.RIO.turretEncoder);
        CANcoderConfiguration turretEncoderConfig = new CANcoderConfiguration();
        MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs();
        magnetConfigs.AbsoluteSensorDiscontinuityPoint = 1;
        magnetConfigs.MagnetOffset = 0.0f;
        magnetConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        turretEncoderConfig.MagnetSensor = magnetConfigs;
        turretEncoder.getConfigurator().apply(turretEncoderConfig);

        turretEncoderOffset = TurretConstants.turretEncoderOffset;

        configurePID();

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        resetMotorToAbsolute();

    }

    @Override
    public void update() {

        outputs.motorVelocityRPS = turretMotor.getVelocity().getValueAsDouble();
        outputs.motorAppliedVoltage = turretMotor.getMotorVoltage().getValueAsDouble();;
        outputs.motorSupplyAmps = turretMotor.getSupplyCurrent().getValueAsDouble();
        outputs.motorTorqueAmps = turretMotor.getTorqueCurrent().getValueAsDouble();

        outputs.motorRotation = Rotation2d.fromRotations(turretMotor.getRotorPosition().getValueAsDouble());
        outputs.encoderRotation = Rotation2d.fromRotations(turretEncoder.getAbsolutePosition().getValueAsDouble());
        outputs.turretRotation = Rotation2d.fromRotations(turretEncoder.getAbsolutePosition().getValueAsDouble()).minus(turretEncoderOffset);
        outputs.turretRotationalVelocityRadPerSec = turretEncoder.getVelocity().getValueAsDouble() * 2 * Math.PI;
        outputs.targetPosition = targetPosition;
        outputs.distanceFromSetpoint = outputs.targetPosition.minus(outputs.turretRotation);

        updatePIDValuesFromNetworkTables();
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
            this.turretEncoder.getAbsolutePosition().getValueAsDouble() 
            - this.turretEncoderOffset.getRotations()) * SwerveDriveConstants.turnGearRatio);
    }

    public TurretOutputs getOutputs() {return outputs;};
}
