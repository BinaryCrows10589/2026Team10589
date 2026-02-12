package binarycrows.robot.SeasonCode.SubStateManagers.Turret;

import binarycrows.robot.SeasonCode.Constants.TurretConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
    public TurretOutputs outputs;

    private double motorVoltage = 0;
    private Rotation2d targetPosition = Rotation2d.kZero;

    private PIDController turretPIDController;

    private boolean inVoltageControlMode = true;

    private DCMotorSim turretMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 
            0.001, 
            TurretConstants.motorToTurretGearRatio
            ), 
            DCMotor.getKrakenX60(1));
    
    public TurretIOSim(TurretOutputs outputs) {
        this.outputs = outputs;
        turretPIDController = new PIDController(
            TurretConstants.turretPIDValueP,
            TurretConstants.turretPIDValueI,
            TurretConstants.turretPIDValueD,
            0.02);
    }

    @Override
    public void update() {
        turretMotor.update(0.02);

        if (!inVoltageControlMode) {
            motorVoltage = MathUtil.clamp(this.turretPIDController.calculate(
                this.turretMotor.getAngularPositionRotations(), this.targetPosition.getRotations()), 
                -TurretConstants.maximumVoltage, TurretConstants.maximumVoltage);
        }

        outputs.motorVelocityRPS = turretMotor.getAngularVelocityRPM() / 60.0;
        outputs.motorAppliedVoltage = motorVoltage;
        outputs.motorSupplyAmps = turretMotor.getCurrentDrawAmps();
        outputs.motorTorqueAmps = outputs.motorSupplyAmps;

        outputs.motorRotation = Rotation2d.fromRotations(turretMotor.getAngularPositionRotations());
        outputs.encoderRotation = Rotation2d.fromRotations(turretMotor.getAngularPositionRotations() / TurretConstants.motorToTurretGearRatio);
        outputs.turretRotation = Rotation2d.fromRotations(turretMotor.getAngularPositionRotations() / TurretConstants.motorToTurretGearRatio);
        outputs.turretRotationalVelocityRadPerSec = turretMotor.getAngularVelocityRadPerSec();
        outputs.targetPosition = targetPosition;
        outputs.distanceFromSetpoint = outputs.targetPosition.minus(outputs.turretRotation);

        turretMotor.setInputVoltage(motorVoltage);
    }

    @Override
    public void setRotorVoltage(double voltage) {
        motorVoltage = voltage;
    }

    @Override
    public void setTargetPosition(Rotation2d position) {
        inVoltageControlMode = false;
        targetPosition = position;
    }

    public TurretOutputs getOutputs() {return outputs;};
}
