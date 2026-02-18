package binarycrows.robot.SeasonCode.SubStateManagers.Hood;

import binarycrows.robot.SeasonCode.Constants.HoodConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodSim implements HoodIO {
    public HoodOutputs outputs;

    private double motorVoltage = 0;
    private Rotation2d targetPosition = Rotation2d.kZero;

    private PIDController hoodPIDController;

    private boolean inVoltageControlMode = true;

    private DCMotorSim hoodMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getMinion(1), 
            0.001, 
            HoodConstants.motorToHoodGearRatio
            ), 
            DCMotor.getMinion(1));
    
    public HoodSim(HoodOutputs outputs) {
        this.outputs = outputs;
        hoodPIDController = new PIDController(
            HoodConstants.hoodPIDValueP,
            HoodConstants.hoodPIDValueI,
            HoodConstants.hoodPIDValueD,
            0.02);
    }

    @Override
    public void update() {
        hoodMotor.update(0.02);

        if (!inVoltageControlMode) {
            motorVoltage = MathUtil.clamp(this.hoodPIDController.calculate(
                this.hoodMotor.getAngularPositionRotations(), this.targetPosition.getRotations()), 
                -HoodConstants.maximumVoltage, HoodConstants.maximumVoltage);
        }

        outputs.motorVelocityRPS = hoodMotor.getAngularVelocityRPM() / 60.0;
        outputs.motorAppliedVoltage = motorVoltage;
        outputs.motorSupplyAmps = hoodMotor.getCurrentDrawAmps();
        outputs.motorTorqueAmps = outputs.motorSupplyAmps;

        outputs.motorRotation = Rotation2d.fromRotations(hoodMotor.getAngularPositionRotations());
        outputs.encoderRotation = Rotation2d.fromRotations(hoodMotor.getAngularPositionRotations() / HoodConstants.motorToHoodGearRatio);
        outputs.hoodRotation = Rotation2d.fromRotations(hoodMotor.getAngularPositionRotations() / HoodConstants.motorToHoodGearRatio);
        outputs.hoodRotationalVelocityRadPerSec = hoodMotor.getAngularVelocityRadPerSec();
        outputs.targetPosition = targetPosition;
        outputs.distanceFromSetpoint = outputs.targetPosition.minus(outputs.hoodRotation);

        hoodMotor.setInputVoltage(motorVoltage);
    }

    @Override
    public void setRotorVoltage(double voltage) {
        inVoltageControlMode = true;
        motorVoltage = voltage;
    }

    @Override
    public void setTargetPosition(Rotation2d position) {
        inVoltageControlMode = false;
        targetPosition = position;
    }

    @Override
    public HoodOutputs getOutputs() {
        return this.outputs;
    }
}
