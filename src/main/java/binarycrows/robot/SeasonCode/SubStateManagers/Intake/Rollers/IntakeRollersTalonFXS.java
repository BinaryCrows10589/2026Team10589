package binarycrows.robot.SeasonCode.SubStateManagers.Intake.Rollers;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.FlywheelConstants;
import binarycrows.robot.SeasonCode.Constants.IntakeConstants;

public class IntakeRollersTalonFXS implements IntakeRollersIO {

    public IntakeRollersOutputs outputs;
    
    private TalonFXS leftMotor;
    private TalonFXS rightMotor;

    private VoltageOut masterMotorVoltageRequest = new VoltageOut(0);

    public IntakeRollersTalonFXS(IntakeRollersOutputs outputs) {
        this.outputs = outputs;

        // Left Motor
        leftMotor = new TalonFXS(CANIDs.RIO.leftIntakeRoller);

        TalonFXSConfiguration masterMotorConfig = new TalonFXSConfiguration();
        masterMotorConfig.MotorOutput.Inverted = IntakeConstants.Rollers.masterMotorInverted;
        masterMotorConfig.MotorOutput.NeutralMode = IntakeConstants.Rollers.motorNeutralMode;

        this.leftMotor.getVelocity().setUpdateFrequency(20);
        this.leftMotor.getAcceleration().setUpdateFrequency(20);
        this.leftMotor.getPosition().setUpdateFrequency(20);
        this.leftMotor.getTorqueCurrent().setUpdateFrequency(50);

        masterMotorConfig.Voltage.PeakForwardVoltage = IntakeConstants.Rollers.maxMotorVoltage;
        masterMotorConfig.Voltage.PeakReverseVoltage = -IntakeConstants.Rollers.maxMotorVoltage;

        this.leftMotor.getConfigurator().apply(masterMotorConfig);
        
        // Right Motor
        rightMotor = new TalonFXS(CANIDs.RIO.rightIntakeRoller);

        this.rightMotor.getVelocity().setUpdateFrequency(20);
        this.rightMotor.getAcceleration().setUpdateFrequency(20);
        this.rightMotor.getPosition().setUpdateFrequency(20);
        this.rightMotor.getTorqueCurrent().setUpdateFrequency(50);

        this.rightMotor.getConfigurator().apply(masterMotorConfig);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), IntakeConstants.Rollers.isSlaveReversed));
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
