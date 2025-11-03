package binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import binarycrows.robot.Utils.LogIOInputs;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroPigeonIO implements GyroIO {
    private Pigeon2 gyro;
    private Rotation2d previousGyroValue;
    public Rotation2d yawAngle;

    public GyroPigeonIO() {
        configureGyro();
    }

    private void configureGyro() {
        this.gyro = new Pigeon2(CANIDs.gyro, SwerveDriveConstants.CANLoopName);
        this.gyro.getConfigurator().apply(new Pigeon2Configuration());
        this.gyro.reset();
        yawAngle = this.gyro.getRotation2d();

        previousGyroValue = gyro.getRotation2d();
    } 

    public void update() {

        yawAngle = this.gyro.getRotation2d();
        
        if (Math.abs(previousGyroValue.minus(yawAngle).getDegrees()) > SwerveDriveConstants.maxAngleDeltaPerFrameDegrees) {
            this.gyro.setYaw(previousGyroValue.getDegrees());
        } else {
            previousGyroValue = yawAngle;
        }

    }

    @Override
    public void resetAngle(Rotation2d newZero) {
        this.gyro.setYaw(newZero.getDegrees());
        previousGyroValue = gyro.getRotation2d();
    }
}
