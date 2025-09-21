package binarycrows.robot.SeasonCode.Subsystems.Elevator;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import binarycrows.robot.SeasonCode.Constants.ElevatorConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.Utils.ConversionUtils;
import binarycrows.robot.Utils.Tuning.RuntimeTunablePIDValues;
import edu.wpi.first.wpilibj.Timer;

@SuppressWarnings("unused")
public class ElevatorDualKrakenIO implements ElevatorIO {
    private TalonFX elevatorMasterMotor;
    private TalonFX elevatorSlaveMotor;

    private CANcoder elevatorEncoder;

    private MotionMagicVoltage desiredElevatorPosition = new MotionMagicVoltage(ElevatorConstants.defaultElevatorPosition);

    public double elevatorRawPosition = 0.0;
    public double elevatorOffsetPosition = 0.0;
    public double rawDesiredElevatorPosition = ElevatorConstants.defaultElevatorPosition;
    public double offsetDesiredElevatorPosition = ElevatorConstants.defaultElevatorPosition;
    public double positionError = 0.0;


    public double elevatorMasterRPM = 0.0;
    public double elevatorMasterAppliedVolts = 0.0;
    public double[] elevatorMasterCurrentAmps = new double[] {};
    public double elevatorMasterMotorTemperatureC = 0;

    public double elevatorSlaveRPM = 0.0;
    public double elevatorSlaveAppliedVolts = 0.0;
    public double[] elevatorSlaveCurrentAmps = new double[] {};
    public double elevatorSlaveMotorTemperatureC = 0;

    private RuntimeTunablePIDValues elevatorMotorPIDConstantTuner;
    private boolean goingDown = false;

    public ElevatorDualKrakenIO() {
        this.elevatorMasterMotor = new TalonFX(ElevatorConstants.masterMotorCANID);
        this.elevatorSlaveMotor = new TalonFX(ElevatorConstants.slaveMotorCANID);
        this.elevatorEncoder = new CANcoder(ElevatorConstants.encoderCANID);

        configureElevatorMotors();

        Timer.delay(.2); // TODO: Maybe one day, things will be better, and we won't have to do this crap
    }

    private void configureElevatorMotors() {
        TalonFXConfiguration masterConfiguration = new TalonFXConfiguration();
        masterConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        masterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        masterConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        masterConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.forwardSoftLimit + ElevatorConstants.encoderOffset;
        masterConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        masterConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.reverseSoftLimit + ElevatorConstants.encoderOffset;
        masterConfiguration.Feedback.SensorToMechanismRatio = 1.0;
        masterConfiguration.Feedback.RotorToSensorRatio = ElevatorConstants.gearRatio;
        masterConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        masterConfiguration.Feedback.FeedbackRemoteSensorID = ElevatorConstants.encoderCANID;
        masterConfiguration.Voltage.PeakForwardVoltage = ElevatorConstants.maxVoltage;
        masterConfiguration.Voltage.PeakReverseVoltage = -ElevatorConstants.maxVoltage;
        masterConfiguration.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.motionMagicCruiseVelocity;
        masterConfiguration.MotionMagic.MotionMagicAcceleration = ElevatorConstants.motionMagicAcceleration;
        masterConfiguration.MotionMagic.MotionMagicJerk = ElevatorConstants.motionMagicJerk;

        
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoderConfig.MagnetSensor.MagnetOffset = 0.0;
        elevatorEncoder.getConfigurator().apply(encoderConfig);
        
        //masterConfiguration.Feedback.FeedbackRotorOffset = elevatorEncoder.getAbsoluteEncoder().getPosition(); // Reset the builtin encoder to the REV encoder's value

        Slot0Configs elevatorPositionalPIDConfigs = new Slot0Configs();
        elevatorPositionalPIDConfigs.kP = ElevatorConstants.PPIDValue;
        elevatorPositionalPIDConfigs.kI = ElevatorConstants.IPIDValue;
        elevatorPositionalPIDConfigs.kD = ElevatorConstants.DPIDValue;
        elevatorPositionalPIDConfigs.kG = ElevatorConstants.GPIDValue;
        elevatorPositionalPIDConfigs.kS = ElevatorConstants.SPIDValue;
        elevatorPositionalPIDConfigs.kV = ElevatorConstants.VPIDValue;
        elevatorPositionalPIDConfigs.kA = ElevatorConstants.APIDValue;
        
        MotionMagicConfigs elevatorMotionMagicConfigs = new MotionMagicConfigs();
        elevatorMotionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.motionMagicCruiseVelocity;
        elevatorMotionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.motionMagicAcceleration;
        elevatorMotionMagicConfigs.MotionMagicJerk = ElevatorConstants.motionMagicJerk;

        this.elevatorMotorPIDConstantTuner = new RuntimeTunablePIDValues("Elevator/", 
            elevatorPositionalPIDConfigs.kP,
            elevatorPositionalPIDConfigs.kI,
            elevatorPositionalPIDConfigs.kD,
            0,
            elevatorPositionalPIDConfigs.kG,
            elevatorPositionalPIDConfigs.kS,
            elevatorPositionalPIDConfigs.kV,
            elevatorPositionalPIDConfigs.kA,
            elevatorMotionMagicConfigs.MotionMagicCruiseVelocity,
            elevatorMotionMagicConfigs.MotionMagicAcceleration,
            elevatorMotionMagicConfigs.MotionMagicJerk
            );

        this.elevatorMasterMotor.getConfigurator().apply(masterConfiguration);
        
        this.elevatorSlaveMotor.getConfigurator().apply(masterConfiguration);
        this.elevatorMasterMotor.getConfigurator().apply(elevatorPositionalPIDConfigs); 
        elevatorSlaveMotor.setControl(new Follower(elevatorMasterMotor.getDeviceID(), ElevatorConstants.isSlaveReversed));

    }


    // This should only be called once and should NEVER be called in a non-testing environment
    private void updatePIDValuesFromNetworkTables() {
        double[] currentElevatorPIDValues = this.elevatorMotorPIDConstantTuner.getUpdatedPIDConstants();
        if(this.elevatorMotorPIDConstantTuner.hasAnyPIDValueChanged()) {
            Slot0Configs newElevatorPIDConfigs = new Slot0Configs();
            newElevatorPIDConfigs.kP = currentElevatorPIDValues[0];
            newElevatorPIDConfigs.kI = currentElevatorPIDValues[1];
            newElevatorPIDConfigs.kD = currentElevatorPIDValues[2];
            newElevatorPIDConfigs.kG = currentElevatorPIDValues[4];
            newElevatorPIDConfigs.kS = currentElevatorPIDValues[5];
            newElevatorPIDConfigs.kV = currentElevatorPIDValues[6];
            newElevatorPIDConfigs.kA = currentElevatorPIDValues[7];

            MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
            motionMagicConfigs.MotionMagicCruiseVelocity = currentElevatorPIDValues[8];
            motionMagicConfigs.MotionMagicAcceleration = currentElevatorPIDValues[9];
            motionMagicConfigs.MotionMagicJerk = currentElevatorPIDValues[10];
            
            this.elevatorMasterMotor.getConfigurator().apply(newElevatorPIDConfigs);
            this.elevatorMasterMotor.getConfigurator().apply(motionMagicConfigs);
        }
    }

    @Override
    public void updateInputs() {
        elevatorRawPosition = elevatorMasterMotor.getPosition().getValueAsDouble();
        elevatorOffsetPosition = elevatorMasterMotor.getPosition().getValueAsDouble() - ElevatorConstants.encoderOffset;
        rawDesiredElevatorPosition = desiredElevatorPosition.Position;
        offsetDesiredElevatorPosition = getOffsetDesiredPosition().Position;
        elevatorMasterRPM = elevatorMasterMotor.getVelocity().getValueAsDouble();
        elevatorMasterAppliedVolts = elevatorMasterMotor.getMotorVoltage().getValueAsDouble();
        elevatorMasterCurrentAmps = new double[] {elevatorMasterMotor.getSupplyCurrent().getValueAsDouble()};
        elevatorMasterMotorTemperatureC = this.elevatorMasterMotor.getDeviceTemp().getValueAsDouble();
        elevatorSlaveRPM = elevatorSlaveMotor.getVelocity().getValueAsDouble();
        elevatorSlaveAppliedVolts = elevatorSlaveMotor.getMotorVoltage().getValueAsDouble();
        elevatorSlaveCurrentAmps = new double[] {elevatorSlaveMotor.getSupplyCurrent().getValueAsDouble()};
        elevatorSlaveMotorTemperatureC = this.elevatorSlaveMotor.getDeviceTemp().getValueAsDouble();
        positionError = getOffsetDesiredPosition().Position - elevatorRawPosition;
    
        if (!MetaConstants.inProduction) updatePIDValuesFromNetworkTables();

        if (this.goingDown) {
            if (ConversionUtils.getIsInTolerance(positionError, 0, ElevatorConstants.catchTolerance)) {
                if(desiredElevatorPosition.Position == ElevatorConstants.elevatorPositions.get(ElevatorStateRequest.BASEMENT)) {
                    elevatorMasterMotor.stopMotor();
                } else {
                   this.elevatorMasterMotor.setControl(getOffsetDesiredPosition());
                }
                goingDown = false;
            } else {
                //double desiredVoltage = MathUtil.clamp(Math.pow(positionError, 3), -1, 0);            
                this.elevatorMasterMotor.setVoltage(-3.25);
            }
        }
    }

    private PositionVoltage getOffsetDesiredPosition() {
        return new PositionVoltage(desiredElevatorPosition.Position + ElevatorConstants.encoderOffset);
    }

    public void disableElevatorMotors() {
        this.elevatorMasterMotor.disable();
    }

    @Override
    public void setDesiredPosition(double desiredPosition) {
        desiredElevatorPosition.Position = desiredPosition;
        this.positionError = (desiredElevatorPosition.Position + ElevatorConstants.encoderOffset) - this.elevatorMasterMotor.getPosition().getValueAsDouble();
        if (positionError < 0 || ConversionUtils.getIsInTolerance(desiredElevatorPosition.Position + ElevatorConstants.encoderOffset, ElevatorConstants.encoderOffset, ElevatorConstants.catchTolerance)) {
            this.goingDown = true;
        } else {
            this.elevatorMasterMotor.setControl(getOffsetDesiredPosition());
            this.goingDown = false;
        }
    }
    
    @Override
    public void incrementDesiredPosition(double increment) {
        desiredElevatorPosition.Position += increment;
        this.elevatorMasterMotor.setControl(getOffsetDesiredPosition());
    }
}
