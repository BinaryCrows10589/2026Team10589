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
    
    private TalonFXS motor;

    private VoltageOut motorVoltageRequest = new VoltageOut(0);

    public IntakeRollersTalonFXS(IntakeRollersOutputs outputs) {
        this.outputs = outputs;

        // Left Motor
        motor = new TalonFXS(CANIDs.RIO.leftIntakeRoller);

        TalonFXSConfiguration motorConfig = new TalonFXSConfiguration();
        motorConfig.MotorOutput.Inverted = IntakeConstants.Rollers.masterMotorInverted;
        motorConfig.MotorOutput.NeutralMode = IntakeConstants.Rollers.motorNeutralMode;

        this.motor.getVelocity().setUpdateFrequency(20);
        this.motor.getAcceleration().setUpdateFrequency(20);
        this.motor.getPosition().setUpdateFrequency(20);
        this.motor.getTorqueCurrent().setUpdateFrequency(50);

        motorConfig.Voltage.PeakForwardVoltage = IntakeConstants.Rollers.maxMotorVoltage;
        motorConfig.Voltage.PeakReverseVoltage = -IntakeConstants.Rollers.maxMotorVoltage;

        this.motor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void setRotorVoltage(double rotorVoltage) {
        motorVoltageRequest = new VoltageOut(rotorVoltage);
        motor.setControl(motorVoltageRequest);
    }

    @Override
    public void update() {
        outputs.leftMotorVelocityRPS = motor.getVelocity().getValueAsDouble();
        outputs.leftMotorAppliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        outputs.leftMotorSupplyAmps = motor.getSupplyCurrent().getValueAsDouble();
        outputs.leftMotorTorqueAmps = motor.getTorqueCurrent().getValueAsDouble();

        outputs.rightMotorVelocityRPS = 0;
        outputs.rightMotorAppliedVoltage = 0;
        outputs.rightMotorSupplyAmps = 0;
        outputs.rightMotorTorqueAmps = 0;

    }
}
