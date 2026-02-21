package binarycrows.robot.SeasonCode.SubStateManagers.Flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.FlywheelConstants;

public class FlywheelTalonFX implements FlywheelIO {

    public FlywheelOutputs outputs;
    
    private TalonFX leftMotor;
    private TalonFX rightMotor;

    private VoltageOut masterMotorVoltageRequest = new VoltageOut(0);

    public FlywheelTalonFX(FlywheelOutputs outputs) {
        this.outputs = outputs;

        // Left Motor
        leftMotor = new TalonFX(CANIDs.RIO.leftFlywheelMotor);

        TalonFXConfiguration masterMotorConfig = new TalonFXConfiguration();
        masterMotorConfig.MotorOutput.Inverted = FlywheelConstants.masterMotorInverted;
        masterMotorConfig.MotorOutput.NeutralMode = FlywheelConstants.motorNeutralMode;

        this.leftMotor.getVelocity().setUpdateFrequency(20);
        this.leftMotor.getAcceleration().setUpdateFrequency(20);
        this.leftMotor.getPosition().setUpdateFrequency(20);
        this.leftMotor.getTorqueCurrent().setUpdateFrequency(50);

        // TODO: You will not beleive it but same note again. 
        // Espeicly with the two motors in case of inversion issues
        masterMotorConfig.Voltage.PeakForwardVoltage = FlywheelConstants.maxMotorVoltage;
        masterMotorConfig.Voltage.PeakReverseVoltage = -FlywheelConstants.maxMotorVoltage;

        this.leftMotor.getConfigurator().apply(masterMotorConfig);
        
        // Right Motor
        rightMotor = new TalonFX(CANIDs.RIO.rightFlywheelMotor);

        this.rightMotor.getVelocity().setUpdateFrequency(20);
        this.rightMotor.getAcceleration().setUpdateFrequency(20);
        this.rightMotor.getPosition().setUpdateFrequency(20);
        this.rightMotor.getTorqueCurrent().setUpdateFrequency(50);

        this.rightMotor.getConfigurator().apply(masterMotorConfig);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), FlywheelConstants.isSlaveReversed));
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

    }
}
