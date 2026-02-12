package binarycrows.robot.SeasonCode.SubStateManagers.Transit;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.FlywheelConstants;
import binarycrows.robot.SeasonCode.Constants.IntakeConstants;
import binarycrows.robot.SeasonCode.Constants.TransitConstants;

public class TransitTalonFXS implements TransitIO {

    public TransitOutputs outputs;
    
    private TalonFXS leftLongitudinalMotor;
    private TalonFXS rightLongitudinalMotor;
    private TalonFXS leftLatitudinalMotor;
    private TalonFXS rightLatitudinalMotor;

    private VoltageOut longitudinalMasterMotorVoltageRequest = new VoltageOut(0);
    private VoltageOut latitudinalMasterMotorVoltageRequest = new VoltageOut(0);

    public TransitTalonFXS(TransitOutputs outputs) {
        this.outputs = outputs;
        // Left Longitudinal Motor
        leftLongitudinalMotor = new TalonFXS(CANIDs.RIO.leftLongitudinalMotor);

        TalonFXSConfiguration longitudinalMasterMotorConfig = new TalonFXSConfiguration();
        longitudinalMasterMotorConfig.MotorOutput.Inverted = TransitConstants.longitudinalMasterMotorInverted;
        longitudinalMasterMotorConfig.MotorOutput.NeutralMode = TransitConstants.longitudinalMasterMotorNeutralMode;

        this.leftLongitudinalMotor.getVelocity().setUpdateFrequency(20);
        this.leftLongitudinalMotor.getAcceleration().setUpdateFrequency(20);
        this.leftLongitudinalMotor.getPosition().setUpdateFrequency(20);
        this.leftLongitudinalMotor.getTorqueCurrent().setUpdateFrequency(50);

        longitudinalMasterMotorConfig.Voltage.PeakForwardVoltage = TransitConstants.maxLongitudinalMotorVoltage;
        longitudinalMasterMotorConfig.Voltage.PeakReverseVoltage = -TransitConstants.maxLongitudinalMotorVoltage;

        this.leftLongitudinalMotor.getConfigurator().apply(longitudinalMasterMotorConfig);
        
        // Right Longitudinal Motor
        rightLongitudinalMotor = new TalonFXS(CANIDs.RIO.rightLongitudinalMotor);

        this.rightLongitudinalMotor.getVelocity().setUpdateFrequency(20);
        this.rightLongitudinalMotor.getAcceleration().setUpdateFrequency(20);
        this.rightLongitudinalMotor.getPosition().setUpdateFrequency(20);
        this.rightLongitudinalMotor.getTorqueCurrent().setUpdateFrequency(50);

        this.rightLongitudinalMotor.getConfigurator().apply(longitudinalMasterMotorConfig);

        rightLongitudinalMotor.setControl(new Follower(leftLongitudinalMotor.getDeviceID(), TransitConstants.isLongitudinalSlaveReversed));

        // Left Latitudinal Motor
        leftLatitudinalMotor = new TalonFXS(CANIDs.RIO.leftLatitudinalMotor);

        TalonFXSConfiguration latitudinalMasterMotorConfig = new TalonFXSConfiguration();
        latitudinalMasterMotorConfig.MotorOutput.Inverted = TransitConstants.latitudinalMasterMotorInverted;
        latitudinalMasterMotorConfig.MotorOutput.NeutralMode = TransitConstants.latitudinalMasterMotorNeutralMode;

        this.leftLatitudinalMotor.getVelocity().setUpdateFrequency(20);
        this.leftLatitudinalMotor.getAcceleration().setUpdateFrequency(20);
        this.leftLatitudinalMotor.getPosition().setUpdateFrequency(20);
        this.leftLatitudinalMotor.getTorqueCurrent().setUpdateFrequency(50);

        latitudinalMasterMotorConfig.Voltage.PeakForwardVoltage = TransitConstants.maxLatitudinalMotorVoltage;
        latitudinalMasterMotorConfig.Voltage.PeakReverseVoltage = -TransitConstants.maxLatitudinalMotorVoltage;

        this.leftLatitudinalMotor.getConfigurator().apply(latitudinalMasterMotorConfig);
        
        // Right Longitudinal Motor
        rightLatitudinalMotor = new TalonFXS(CANIDs.RIO.rightLatitudinalMotor);

        this.rightLatitudinalMotor.getVelocity().setUpdateFrequency(20);
        this.rightLatitudinalMotor.getAcceleration().setUpdateFrequency(20);
        this.rightLatitudinalMotor.getPosition().setUpdateFrequency(20);
        this.rightLatitudinalMotor.getTorqueCurrent().setUpdateFrequency(50);

        this.rightLatitudinalMotor.getConfigurator().apply(latitudinalMasterMotorConfig);

        rightLatitudinalMotor.setControl(new Follower(leftLatitudinalMotor.getDeviceID(), TransitConstants.isLatitudinalSlaveReversed));
    }

    @Override
    public void setLatitudinalVoltage(double rotorVoltage) {
        latitudinalMasterMotorVoltageRequest = new VoltageOut(rotorVoltage);
        leftLatitudinalMotor.setControl(latitudinalMasterMotorVoltageRequest);
    }

    @Override
    public void setLongitudinalVoltage(double rotorVoltage) {
        longitudinalMasterMotorVoltageRequest = new VoltageOut(rotorVoltage);
        leftLongitudinalMotor.setControl(longitudinalMasterMotorVoltageRequest);
    }

    @Override
    public void update() {
        outputs.leftLongitudinalMotorVelocityRPS = leftLongitudinalMotor.getVelocity().getValueAsDouble();
        outputs.leftLongitudinalMotorAppliedVoltage = leftLongitudinalMotor.getMotorVoltage().getValueAsDouble();
        outputs.leftLongitudinalMotorSupplyAmps = leftLongitudinalMotor.getSupplyCurrent().getValueAsDouble();
        outputs.leftLongitudinalMotorTorqueAmps = leftLongitudinalMotor.getTorqueCurrent().getValueAsDouble();

        outputs.rightLongitudinalMotorVelocityRPS = rightLongitudinalMotor.getVelocity().getValueAsDouble();
        outputs.rightLongitudinalMotorAppliedVoltage = rightLongitudinalMotor.getMotorVoltage().getValueAsDouble();
        outputs.rightLongitudinalMotorSupplyAmps = rightLongitudinalMotor.getSupplyCurrent().getValueAsDouble();
        outputs.rightLongitudinalMotorTorqueAmps = rightLongitudinalMotor.getTorqueCurrent().getValueAsDouble();

        outputs.leftLatitudinalMotorVelocityRPS = leftLatitudinalMotor.getVelocity().getValueAsDouble();
        outputs.leftLatitudinalMotorAppliedVoltage = leftLatitudinalMotor.getMotorVoltage().getValueAsDouble();
        outputs.leftLatitudinalMotorSupplyAmps = leftLatitudinalMotor.getSupplyCurrent().getValueAsDouble();
        outputs.leftLatitudinalMotorTorqueAmps = leftLatitudinalMotor.getTorqueCurrent().getValueAsDouble();

        outputs.rightLatitudinalMotorVelocityRPS = rightLatitudinalMotor.getVelocity().getValueAsDouble();
        outputs.rightLatitudinalMotorAppliedVoltage = rightLatitudinalMotor.getMotorVoltage().getValueAsDouble();
        outputs.rightLatitudinalMotorSupplyAmps = rightLatitudinalMotor.getSupplyCurrent().getValueAsDouble();
        outputs.rightLatitudinalMotorTorqueAmps = rightLatitudinalMotor.getTorqueCurrent().getValueAsDouble();

    }
}
