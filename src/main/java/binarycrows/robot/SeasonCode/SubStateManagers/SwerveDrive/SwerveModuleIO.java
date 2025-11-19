package binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import binarycrows.robot.Utils.Tuning.RuntimeTunablePIDValues;
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
    
        public double turnMotorAbsolutePositionRotations = 0.0;
        public double turnMotorRelativePositionRotations = 0.0;
        public double turnMotorRPS = 0.0;
        public double turnMotorAppliedVolts = 0.0;
        public double turnMotorSupplyAmps = 0.0;
        public double turnMotorTorqueAmps = 0.0;
    }

    public void setDesiredModuleDriveVoltage(double desiredVoltage);

    public void setDesiredModuleAngle(Rotation2d desiredModuleAngle);

    public void resetTurningMotorToAbsolute();

    public SwerveModuleOutputs getOutputs();

    public void updateOutputs();
}
