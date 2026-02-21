package binarycrows.robot.SeasonCode.SubStateManagers.Climber;

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
import binarycrows.robot.SeasonCode.Constants.ClimberConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.ClimberConstants;
import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import binarycrows.robot.Utils.Tuning.RuntimeTunablePIDValues;
import edu.wpi.first.math.geometry.Rotation2d;

public class ClimberTalonFX implements ClimberIO {

    public ClimberOutputs outputs;

    private Rotation2d targetPosition = Rotation2d.kZero;
    
    private TalonFX motor;

    private VoltageOut voltageRequest = new VoltageOut(0);
    private PositionDutyCycle controlRequest = new PositionDutyCycle(0);

    private RuntimeTunablePIDValues climberPIDConstantTuner;

    public ClimberTalonFX(ClimberOutputs outputs) {
        this.outputs = outputs;

        // Left Motor
        motor = new TalonFX(CANIDs.RIO.climberMotor);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = ClimberConstants.motorInverted;
        motorConfig.MotorOutput.NeutralMode = ClimberConstants.motorNeutralMode;

        this.motor.getVelocity().setUpdateFrequency(20);
        this.motor.getAcceleration().setUpdateFrequency(20);
        this.motor.getPosition().setUpdateFrequency(20);
        this.motor.getTorqueCurrent().setUpdateFrequency(50);

        // TODO: Same note again. Also I thought you were not having a absolute encdoer for the climber?
        motorConfig.Voltage.PeakForwardVoltage = ClimberConstants.maxMotorVoltageUp;
        motorConfig.Voltage.PeakReverseVoltage = ClimberConstants.maxMotorVoltageDown;

        this.motor.getConfigurator().apply(motorConfig);

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
        voltageRequest = new VoltageOut(rotorVoltage);
        motor.setControl(voltageRequest);
    }

    @Override
    public void update() {
        outputs.motorVelocityRPS = motor.getVelocity().getValueAsDouble();
        outputs.motorAppliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        outputs.motorSupplyAmps = motor.getSupplyCurrent().getValueAsDouble();
        outputs.motorTorqueAmps = motor.getTorqueCurrent().getValueAsDouble();

        outputs.motorRotation = Rotation2d.fromRotations(motor.getRotorPosition().getValueAsDouble());
        outputs.targetPosition = targetPosition;
        outputs.distanceFromSetpoint = outputs.targetPosition.minus(outputs.motorRotation);

        updatePIDValuesFromNetworkTables();


    }

    public void updatePIDValuesFromNetworkTables() {


        if (MetaConstants.inProduction) return; // Don't run at competitions!
        if(this.climberPIDConstantTuner.hasAnyPIDValueChanged()) {
            this.motor.getConfigurator().apply(climberPIDConstantTuner.generatePIDFFConfigs());
        }

    }

    private void configurePID() {
        Slot0Configs turretPIDConfig = new Slot0Configs();

        turretPIDConfig.kP = ClimberConstants.climberPIDValueP;
        turretPIDConfig.kI = ClimberConstants.climberPIDValueI;
        turretPIDConfig.kD = ClimberConstants.climberPIDValueD;
        turretPIDConfig.kS = ClimberConstants.climberPIDValueFF;

        this.motor.getConfigurator().apply(turretPIDConfig);

        climberPIDConstantTuner = new RuntimeTunablePIDValues("Climber/PIDValues",
        ClimberConstants.climberPIDValueP, ClimberConstants.climberPIDValueI, ClimberConstants.climberPIDValueD, ClimberConstants.climberPIDValueFF);

    }

    @Override
    public void setTargetPosition(Rotation2d position) {
        targetPosition = position;
        controlRequest = new PositionDutyCycle(targetPosition.getRotations());
        motor.setControl(controlRequest);
    }
}
