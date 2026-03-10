package binarycrows.robot.SeasonCode.SubStateManagers.Transit;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.TransitConstants;

public class TransitTalonFXS implements TransitIO {

    public TransitOutputs outputs;
    
    private TalonFXS longitudinalMotor;
    private TalonFXS leftLatitudinalMotor;
    private TalonFXS rightLatitudinalMotor;
    private TalonFXS inAndUpMotor;

    private VoltageOut longitudinalMotorVoltageRequest = new VoltageOut(0);
    private VoltageOut latitudinalMasterMotorVoltageRequest = new VoltageOut(0);
    private VoltageOut inAndUpMotorVoltageRequest = new VoltageOut(0);

    public TransitTalonFXS(TransitOutputs outputs) {
        this.outputs = outputs;
        // Left Longitudinal Motor
        longitudinalMotor = new TalonFXS(CANIDs.RIO.longitudinalMotor);

        TalonFXSConfiguration longitudinalMotorConfig = new TalonFXSConfiguration();
        longitudinalMotorConfig.MotorOutput.Inverted = TransitConstants.longitudinalMotorInverted;
        longitudinalMotorConfig.MotorOutput.NeutralMode = TransitConstants.longitudinalMotorNeutralMode;
        longitudinalMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        this.longitudinalMotor.getVelocity().setUpdateFrequency(20);
        this.longitudinalMotor.getAcceleration().setUpdateFrequency(20);
        this.longitudinalMotor.getPosition().setUpdateFrequency(20);
        this.longitudinalMotor.getTorqueCurrent().setUpdateFrequency(50);

        longitudinalMotorConfig.Voltage.PeakForwardVoltage = TransitConstants.maxLongitudinalMotorVoltage;
        longitudinalMotorConfig.Voltage.PeakReverseVoltage = -TransitConstants.maxLongitudinalMotorVoltage;

        this.longitudinalMotor.getConfigurator().apply(longitudinalMotorConfig);

        // Left Latitudinal Motor
        leftLatitudinalMotor = new TalonFXS(CANIDs.RIO.leftLatitudinalMotor);

        TalonFXSConfiguration latitudinalMasterMotorConfig = new TalonFXSConfiguration();
        latitudinalMasterMotorConfig.MotorOutput.Inverted = TransitConstants.latitudinalMasterMotorInverted;
        latitudinalMasterMotorConfig.MotorOutput.NeutralMode = TransitConstants.latitudinalMasterMotorNeutralMode;
        latitudinalMasterMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

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

        // In & Up Motor
        inAndUpMotor = new TalonFXS(CANIDs.RIO.inAndUpMotor);

        TalonFXSConfiguration inAndUpMotorConfig = new TalonFXSConfiguration();
        inAndUpMotorConfig.MotorOutput.Inverted = TransitConstants.inAndUpMotorInverted;
        inAndUpMotorConfig.MotorOutput.NeutralMode = TransitConstants.inAndUpMotorNeutralMode;

        this.inAndUpMotor.getVelocity().setUpdateFrequency(20);
        this.inAndUpMotor.getAcceleration().setUpdateFrequency(20);
        this.inAndUpMotor.getPosition().setUpdateFrequency(20);
        this.inAndUpMotor.getTorqueCurrent().setUpdateFrequency(50);

        inAndUpMotorConfig.Voltage.PeakForwardVoltage = TransitConstants.maxInAndUpMotorVoltage;
        inAndUpMotorConfig.Voltage.PeakReverseVoltage = -TransitConstants.maxInAndUpMotorVoltage;
        inAndUpMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        this.inAndUpMotor.getConfigurator().apply(inAndUpMotorConfig);
    }

    @Override
    public void setLatitudinalVoltage(double rotorVoltage) {
        latitudinalMasterMotorVoltageRequest = new VoltageOut(rotorVoltage);
        leftLatitudinalMotor.setControl(latitudinalMasterMotorVoltageRequest);
    }

    @Override
    public void setLongitudinalVoltage(double rotorVoltage) {
        longitudinalMotorVoltageRequest = new VoltageOut(rotorVoltage);
        longitudinalMotor.setControl(longitudinalMotorVoltageRequest);
    }
    
    @Override
    public void setInAndUpVoltage(double rotorVoltage) {
        inAndUpMotorVoltageRequest = new VoltageOut(rotorVoltage);
        inAndUpMotor.setControl(inAndUpMotorVoltageRequest);
    }

    @Override
    public void update() {
        outputs.longitudinalMotorRequestedVoltage = longitudinalMotorVoltageRequest.Output;
        outputs.longitudinalMotorVelocityRPS = longitudinalMotor.getVelocity().getValueAsDouble();
        outputs.longitudinalMotorAppliedVoltage = longitudinalMotor.getMotorVoltage().getValueAsDouble();
        outputs.longitudinalMotorSupplyAmps = longitudinalMotor.getSupplyCurrent().getValueAsDouble();
        outputs.longitudinalMotorTorqueAmps = longitudinalMotor.getTorqueCurrent().getValueAsDouble();

        outputs.leftLatitudinalMotorRequestedVoltage = latitudinalMasterMotorVoltageRequest.Output;
        outputs.leftLatitudinalMotorVelocityRPS = leftLatitudinalMotor.getVelocity().getValueAsDouble();
        outputs.leftLatitudinalMotorAppliedVoltage = leftLatitudinalMotor.getMotorVoltage().getValueAsDouble();
        outputs.leftLatitudinalMotorSupplyAmps = leftLatitudinalMotor.getSupplyCurrent().getValueAsDouble();
        outputs.leftLatitudinalMotorTorqueAmps = leftLatitudinalMotor.getTorqueCurrent().getValueAsDouble();

        outputs.rightLatitudinalMotorRequestedVoltage = latitudinalMasterMotorVoltageRequest.Output;
        outputs.rightLatitudinalMotorVelocityRPS = rightLatitudinalMotor.getVelocity().getValueAsDouble();
        outputs.rightLatitudinalMotorAppliedVoltage = rightLatitudinalMotor.getMotorVoltage().getValueAsDouble();
        outputs.rightLatitudinalMotorSupplyAmps = rightLatitudinalMotor.getSupplyCurrent().getValueAsDouble();
        outputs.rightLatitudinalMotorTorqueAmps = rightLatitudinalMotor.getTorqueCurrent().getValueAsDouble();

        outputs.inAndUpMotorRequestedVoltage = inAndUpMotorVoltageRequest.Output;
        outputs.inAndUpMotorVelocityRPS = inAndUpMotor.getVelocity().getValueAsDouble();
        outputs.inAndUpMotorAppliedVoltage = inAndUpMotor.getMotorVoltage().getValueAsDouble();
        outputs.inAndUpMotorSupplyAmps =  inAndUpMotor.getSupplyCurrent().getValueAsDouble();
        outputs.inAndUpMotorTorqueAmps =  inAndUpMotor.getTorqueCurrent().getValueAsDouble();

    }
}
