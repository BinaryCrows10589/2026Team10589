package binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.HoodConstants;
import binarycrows.robot.SeasonCode.Constants.IntakeConstants;
import binarycrows.robot.SeasonCode.Constants.IntakeConstants.Pivot;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.TurretConstants;
import binarycrows.robot.Utils.Tuning.RuntimeTunablePIDValues;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PivotSim implements PivotIO {

    public PivotOutputs outputs;

    private Rotation2d targetPosition = Rotation2d.kZero;
    boolean usingPID = false;
    
    private DCMotorSim leftMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 
            0.001, 
            IntakeConstants.Pivot.gearRatio
            ), 
            DCMotor.getKrakenX60(1));
    private DCMotorSim rightMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 
            0.001, 
            IntakeConstants.Pivot.gearRatio
            ), 
            DCMotor.getKrakenX60(1));

    private RuntimeTunablePIDValues pivotPIDConstantTuner;

    private PIDController hoodController;

    private double motorVoltage = 0;

    public PivotSim(PivotOutputs outputs) {
        this.outputs = outputs;

        configurePID();
    }

    @Override
    public void setRotorVoltage(double rotorVoltage) {
        motorVoltage = rotorVoltage;
        usingPID = false;
    }

    @Override
    public void update() {
        leftMotor.update(MetaConstants.loopPeriodSeconds);
        rightMotor.update(MetaConstants.loopPeriodSeconds);

        outputs.leftMotorVelocityRPS = leftMotor.getAngularVelocityRPM() / 60.0;
        outputs.leftMotorAppliedVoltage = motorVoltage;
        outputs.leftMotorSupplyAmps = leftMotor.getCurrentDrawAmps();
        outputs.leftMotorTorqueAmps = outputs.leftMotorSupplyAmps;

        outputs.rightMotorVelocityRPS = rightMotor.getAngularVelocityRPM() / 60.0;
        outputs.rightMotorAppliedVoltage = motorVoltage;
        outputs.rightMotorSupplyAmps = rightMotor.getCurrentDrawAmps();
        outputs.rightMotorTorqueAmps = outputs.rightMotorSupplyAmps;

        outputs.motorRotation = Rotation2d.fromRotations(leftMotor.getAngularPositionRotations());
        outputs.encoderRotation = outputs.motorRotation.div(IntakeConstants.Pivot.gearRatio);
        outputs.pivotRotation = outputs.encoderRotation;
        outputs.pivotRotationalVelocityRadPerSec = leftMotor.getAngularVelocityRadPerSec() / IntakeConstants.Pivot.gearRatio;
        outputs.targetPosition = targetPosition;
        outputs.distanceFromSetpoint = outputs.targetPosition.minus(outputs.pivotRotation);

        updatePIDValuesFromNetworkTables();

        if (usingPID) {
            motorVoltage = hoodController.calculate(outputs.pivotRotation.getRadians(), targetPosition.getRadians());
        }
        leftMotor.setInputVoltage(motorVoltage);
        rightMotor.setInputVoltage(motorVoltage * (IntakeConstants.Pivot.isSlaveReversed == MotorAlignmentValue.Opposed ? -1 : 1));

    }

    public void updatePIDValuesFromNetworkTables() {


        if (MetaConstants.inProduction) return; // Don't run at competitions!
        if(this.pivotPIDConstantTuner.hasAnyPIDValueChanged()) {
            double[] newValues = pivotPIDConstantTuner.getUpdatedPIDConstants();
            this.hoodController.setPID(newValues[0], newValues[1], newValues[2]);
        }

    }

    private void configurePID() {

        hoodController = new PIDController(HoodConstants.hoodPIDValueP, HoodConstants.hoodPIDValueI, HoodConstants.hoodPIDValueD);

        pivotPIDConstantTuner = new RuntimeTunablePIDValues("Pivot/PIDValues",
        HoodConstants.hoodPIDValueP, HoodConstants.hoodPIDValueI, HoodConstants.hoodPIDValueD, HoodConstants.hoodPIDValueFF);

    }

    @Override
    public void setTargetPosition(Rotation2d position) {
        targetPosition = position;
        usingPID = true;
    }
}
