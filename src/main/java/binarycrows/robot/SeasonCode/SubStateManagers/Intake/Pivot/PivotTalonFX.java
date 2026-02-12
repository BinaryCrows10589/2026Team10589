package binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.HoodConstants;
import binarycrows.robot.SeasonCode.Constants.IntakeConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.IntakeConstants.Pivot;
import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import binarycrows.robot.Utils.Tuning.RuntimeTunablePIDValues;
import edu.wpi.first.math.geometry.Rotation2d;

public class PivotTalonFX implements PivotIO {

    public PivotOutputs outputs;

    private Rotation2d targetPosition = Rotation2d.kZero;
    
    private TalonFX leftMotor;
    private TalonFX rightMotor;

    private CANcoder pivotEncoder;

    private Rotation2d pivotEncoderOffset;

    private VoltageOut masterMotorVoltageRequest = new VoltageOut(0);
    private PositionDutyCycle pivotControlRequest = new PositionDutyCycle(0);

    private RuntimeTunablePIDValues pivotPIDConstantTuner;


    public PivotTalonFX(PivotOutputs outputs) {
        this.outputs = outputs;

        // Left Motor
        leftMotor = new TalonFX(CANIDs.RIO.leftPivotMotor);

        TalonFXConfiguration masterMotorConfig = new TalonFXConfiguration();
        masterMotorConfig.MotorOutput.Inverted = IntakeConstants.Pivot.masterMotorInverted;
        masterMotorConfig.MotorOutput.NeutralMode = IntakeConstants.Pivot.motorNeutralMode;

        this.leftMotor.getVelocity().setUpdateFrequency(20);
        this.leftMotor.getAcceleration().setUpdateFrequency(20);
        this.leftMotor.getPosition().setUpdateFrequency(20);
        this.leftMotor.getTorqueCurrent().setUpdateFrequency(50);

        masterMotorConfig.Voltage.PeakForwardVoltage = IntakeConstants.Pivot.maxMotorVoltageUp;
        masterMotorConfig.Voltage.PeakReverseVoltage = IntakeConstants.Pivot.maxMotorVoltageDown;

        this.leftMotor.getConfigurator().apply(masterMotorConfig);
        
        // Right Motor
        rightMotor = new TalonFX(CANIDs.RIO.rightPivotMotor);

        this.rightMotor.getVelocity().setUpdateFrequency(20);
        this.rightMotor.getAcceleration().setUpdateFrequency(20);
        this.rightMotor.getPosition().setUpdateFrequency(20);
        this.rightMotor.getTorqueCurrent().setUpdateFrequency(50);

        this.rightMotor.getConfigurator().apply(masterMotorConfig);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), IntakeConstants.Pivot.isSlaveReversed));

        pivotEncoder = new CANcoder(CANIDs.RIO.pivotEncoder);
        CANcoderConfiguration pivotEncoderConfig = new CANcoderConfiguration();
        MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs();
        magnetConfigs.AbsoluteSensorDiscontinuityPoint = 1;
        magnetConfigs.MagnetOffset = 0.0f;
        magnetConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        pivotEncoderConfig.MagnetSensor = magnetConfigs;
        pivotEncoder.getConfigurator().apply(pivotEncoderConfig);

        pivotEncoderOffset = IntakeConstants.Pivot.pivotEncoderOffset;

        configurePID();

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        resetMotorToAbsolute();
    }

    @Override
    public void setRotorVoltage(double rotorVoltage) {
        masterMotorVoltageRequest = new VoltageOut(rotorVoltage);
        leftMotor.setControl(masterMotorVoltageRequest);
    }

    @Override
    public void update() {
        outputs.leftMotorVelocityRPS = leftMotor.getVelocity().getValueAsDouble();
        outputs.leftMotorAppliedVoltage = leftMotor.getMotorVoltage().getValueAsDouble();
        outputs.leftMotorSupplyAmps = leftMotor.getSupplyCurrent().getValueAsDouble();
        outputs.leftMotorTorqueAmps = leftMotor.getTorqueCurrent().getValueAsDouble();

        outputs.rightMotorVelocityRPS = rightMotor.getVelocity().getValueAsDouble();
        outputs.rightMotorAppliedVoltage = rightMotor.getMotorVoltage().getValueAsDouble();
        outputs.rightMotorSupplyAmps = rightMotor.getSupplyCurrent().getValueAsDouble();
        outputs.rightMotorTorqueAmps = rightMotor.getTorqueCurrent().getValueAsDouble();

        outputs.motorRotation = Rotation2d.fromRotations(leftMotor.getRotorPosition().getValueAsDouble());
        outputs.encoderRotation = Rotation2d.fromRotations(pivotEncoder.getAbsolutePosition().getValueAsDouble());
        outputs.pivotRotation = Rotation2d.fromRotations(pivotEncoder.getAbsolutePosition().getValueAsDouble()).minus(pivotEncoderOffset);
        outputs.pivotRotationalVelocityRadPerSec = pivotEncoder.getVelocity().getValueAsDouble() * 2 * Math.PI;
        outputs.targetPosition = targetPosition;
        outputs.distanceFromSetpoint = outputs.targetPosition.minus(outputs.pivotRotation);

        updatePIDValuesFromNetworkTables();


    }

    public void updatePIDValuesFromNetworkTables() {


        if (MetaConstants.inProduction) return; // Don't run at competitions!
        if(this.pivotPIDConstantTuner.hasAnyPIDValueChanged()) {
            this.leftMotor.getConfigurator().apply(pivotPIDConstantTuner.generatePIDFFConfigs());
        }

    }

    private void configurePID() {
        Slot0Configs turretPIDConfig = new Slot0Configs();

        turretPIDConfig.kP = IntakeConstants.Pivot.pivotPIDValueP;
        turretPIDConfig.kI = IntakeConstants.Pivot.pivotPIDValueI;
        turretPIDConfig.kD = IntakeConstants.Pivot.pivotPIDValueD;
        turretPIDConfig.kS = IntakeConstants.Pivot.pivotPIDValueFF;

        this.leftMotor.getConfigurator().apply(turretPIDConfig);

        pivotPIDConstantTuner = new RuntimeTunablePIDValues("Pivot/PIDValues",
        IntakeConstants.Pivot.pivotPIDValueP, IntakeConstants.Pivot.pivotPIDValueI, IntakeConstants.Pivot.pivotPIDValueD, IntakeConstants.Pivot.pivotPIDValueFF);

    }

    @Override
    public void setTargetPosition(Rotation2d position) {
        targetPosition = position;
        pivotControlRequest = new PositionDutyCycle(targetPosition.getRotations());
        leftMotor.setControl(pivotControlRequest);
    }

    @Override
    public void resetMotorToAbsolute() {
        this.leftMotor.setPosition((
            this.pivotEncoder.getAbsolutePosition().getValueAsDouble() 
            - this.pivotEncoderOffset.getRotations()) * IntakeConstants.Pivot.gearRatio);
    }
}
