package binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroPigeonIO implements GyroIO {
    private Pigeon2 gyro;
    private Rotation2d previousGyroValue = Rotation2d.kZero;
    public GyroOutputs outputs;

    public GyroPigeonIO(GyroOutputs outputs) {
        this.outputs = outputs;
        configureGyro();
    }

    private void configureGyro() {
        this.gyro = new Pigeon2(CANIDs.gyro, SwerveDriveConstants.CANLoopName);
        this.gyro.getConfigurator().apply(new Pigeon2Configuration());
        this.gyro.reset();
        updateGyroAngle();

        updatePreviousGyroAngle();
    } 

    public void update() {

        updateGyroAngle();
        updateOutputs();
        
        /*if (Math.abs(previousGyroValue.minus(outputs.yawAngle).getDegrees()) > SwerveDriveConstants.maxAngleDeltaPerFrameDegrees) {
            this.gyro.setYaw(previousGyroValue.getDegrees());
        } else {*/
        previousGyroValue = outputs.yawAngle;
        //}

    }

    public void updateGyroAngle() {
        this.outputs.yawAngle = this.gyro.getRotation2d();
        this.outputs.yawAngleVelocityDegreesPerSecond = this.gyro.getAngularVelocityZDevice().getValueAsDouble();
    }
    public void updateOutputs() {
        this.outputs.xAccelerationMetersPerSecondPerSecond = this.gyro.getAccelerationX().getValueAsDouble() * 9.80665;
        this.outputs.yAccelerationMetersPerSecondPerSecond = this.gyro.getAccelerationY().getValueAsDouble() * 9.80665;

    }
    public void updatePreviousGyroAngle() {
        previousGyroValue = this.gyro.getRotation2d();
    }

    @Override
    public void resetAngle(Rotation2d newZero) {
        this.gyro.setYaw(newZero.getDegrees());
        updatePreviousGyroAngle();
    }
}
