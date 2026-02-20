package binarycrows.robot.SeasonCode.SubStateManagers.Flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.FlywheelConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.TurretConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelSim implements FlywheelIO {

    public FlywheelOutputs outputs;
    
    private DCMotorSim leftMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 
            0.001, 
            FlywheelConstants.gearRatio
            ), 
            DCMotor.getKrakenX60(1));
    private DCMotorSim rightMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 
            0.001, 
            FlywheelConstants.gearRatio
            ), 
            DCMotor.getKrakenX60(1));

    private double motorVoltage = 0;

    public FlywheelSim(FlywheelOutputs outputs) {
        this.outputs = outputs;
    }

    @Override
    public void setRotorVoltage(double rotorVoltage) {
        motorVoltage = rotorVoltage;
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


        leftMotor.setInputVoltage(motorVoltage);
        leftMotor.setInputVoltage(motorVoltage * (FlywheelConstants.isSlaveReversed == MotorAlignmentValue.Opposed ? -1 : 1));

    }
}
