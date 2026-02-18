package binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModuleIO {

    public class SwerveModuleOutputs {
        public double driveMotorRPS = 0.0;
        public double driveMotorSpeedMetersPerSecond = 0.0;
        public double driveMotorDistanceRotations = 0.0;
        public double driveMotorDistanceMeters = 0.0;
        public double driveMotorAppliedVolts = 0.0;
        public double driveMotorSupplyAmps = 0.0;
        public double driveMotorTorqueAmps = 0.0;

        public double initialDistanceMeters = 0.0;
    
        public double turnMotorAbsolutePositionRotations = 0.0;
        public double turnMotorRelativePositionRotations = 0.0;
        public double turnMotorRPS = 0.0;
        public double turnMotorAppliedVolts = 0.0;
        public double turnMotorSupplyAmps = 0.0;
        public double turnMotorTorqueAmps = 0.0;
        public double turnMotorDesiredPositionRotations = 0.0;

    }

    public void setDesiredModuleDriveVoltage(double desiredVoltage);

    public void setDesiredModuleAngle(Rotation2d desiredModuleAngle);

    public void resetTurningMotorToAbsolute();

    public SwerveModuleOutputs getOutputs();

    public void update();
}
