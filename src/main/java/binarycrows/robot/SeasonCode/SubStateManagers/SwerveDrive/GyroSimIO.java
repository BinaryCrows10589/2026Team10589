package binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class GyroSimIO implements GyroIO {
    double lastInputsUpdateTime = Timer.getFPGATimestamp();

    public GyroOutputs outputs;
    
    public GyroSimIO(GyroOutputs outputs) {
        this.outputs = outputs;
    }

    @Override
    public void update() {
        // leave outputs at their defaults (for now)
    }
}
