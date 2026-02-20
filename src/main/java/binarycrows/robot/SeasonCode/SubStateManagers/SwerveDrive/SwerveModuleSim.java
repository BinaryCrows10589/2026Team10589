package binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive;

import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SwerveModuleSim implements SwerveModuleIO {


    LinearSystem<N2, N1, N2> driveMotorLinearSystem = edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.00148, 1);
    LinearSystem<N2, N1, N2> turnMotorLinearSystem = edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.00148, 1);

    private DCMotorSim driveMotor = new DCMotorSim(driveMotorLinearSystem, DCMotor.getKrakenX60(1));
    private DCMotorSim turnMotor = new DCMotorSim(turnMotorLinearSystem, DCMotor.getKrakenX60(1));

    //private PIDController drivePIDController; 
    private PIDController turnPIDController; 
    
    //private double desiredVelocityRPM = 0;
    private double desiredPositionRotations = 0;
    private double driveVolts = 0;

    private SwerveModuleOutputs outputs;

    @SuppressWarnings("unused")
    private String swerveModuleName;

    public SwerveModuleSim(String swerveModuleName, SwerveModuleOutputs outputs) {
        this.swerveModuleName = swerveModuleName;
        this.outputs = outputs;
        configTurnPID();
    }

    private void configTurnPID() {
        this.turnPIDController = new PIDController(SwerveDriveConstants.turnPIDValueP,
            SwerveDriveConstants.turnPIDValueI,
            SwerveDriveConstants.turnPIDValueD, MetaConstants.loopPeriodSeconds);
    }

    @Override
    public void update() {
        driveMotor.update(MetaConstants.loopPeriodSeconds);
        turnMotor.update(MetaConstants.loopPeriodSeconds);
        

        double turnVolts = MathUtil.clamp(this.turnPIDController.calculate(this.turnMotor.getAngularPositionRotations(), this.desiredPositionRotations), -12, 12);
        outputs.driveMotorRPS = this.driveMotor.getAngularVelocityRPM() / 60.0;
        outputs.driveMotorSpeedMetersPerSecond = outputs.driveMotorRPS * 60 * SwerveDriveConstants.driveConversionVelocityFactor;
        outputs.driveMotorDistanceRotations = this.driveMotor.getAngularPositionRotations();
        outputs.driveMotorDistanceMeters = (outputs.driveMotorDistanceRotations / SwerveDriveConstants.driveGearRatio) * SwerveDriveConstants.wheelDistancePerRotation;
        outputs.driveMotorAppliedVolts = driveVolts;
        outputs.driveMotorSupplyAmps = this.driveMotor.getCurrentDrawAmps();
        outputs.driveMotorTorqueAmps = outputs.driveMotorTorqueAmps; // Not a valid value from sim

        outputs.turnMotorRPS = this.turnMotor.getAngularVelocityRPM() / 60.0;
        outputs.turnMotorAbsolutePositionRotations = this.turnMotor.getAngularPositionRotations();
        outputs.turnMotorRelativePositionRotations = outputs.turnMotorAbsolutePositionRotations; // Same as absolute, since there's no offset
        outputs.turnMotorAppliedVolts = turnVolts;
        outputs.turnMotorSupplyAmps = this.turnMotor.getCurrentDrawAmps();
        outputs.turnMotorTorqueAmps = outputs.turnMotorTorqueAmps; // Not a valid value from sim
        outputs.turnMotorDesiredPositionRotations = this.desiredPositionRotations;
        
        this.driveMotor.setInputVoltage(this.driveVolts);
        this.turnMotor.setInputVoltage(turnVolts);
    }

    @Override
    public void setDesiredModuleDriveVoltage(double desiredVoltage) {
        this.driveVolts = desiredVoltage;
    }

    @Override
    public void setDesiredModuleAngle(Rotation2d desiredModuleAngle) {
        double desiredModuleRotations = desiredModuleAngle.getRotations(); 

        this.desiredPositionRotations = desiredModuleRotations;

    }

    @Override
    public SwerveModuleOutputs getOutputs() {
        return outputs;
    }

    @Override
    public void resetTurningMotorToAbsolute() {
        // no-op
    }

    
}
