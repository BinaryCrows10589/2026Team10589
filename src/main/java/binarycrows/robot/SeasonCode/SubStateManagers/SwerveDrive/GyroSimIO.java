package binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class GyroSimIO implements GyroIO {
    double lastInputsUpdateTime = Timer.getFPGATimestamp();

    public GyroOutputs outputs;
    
    public GyroSimIO(GyroOutputs outputs) {
        this.outputs = outputs;
    }

    @Override
    public void update() {
        double currentTime = Timer.getFPGATimestamp();  
        double deltaTimeSeconds = currentTime - this.lastInputsUpdateTime;
        this.lastInputsUpdateTime = currentTime;
        double rotationSpeedRadPerSecond = DriveSubStateManager.getInstance().getChassisSpeeds().omegaRadiansPerSecond;

        outputs.yawAngle = Rotation2d.fromRadians(((rotationSpeedRadPerSecond * deltaTimeSeconds)) + outputs.yawAngle.getRadians()); // Add to the current rotation value by the rotation rate
        outputs.yawAngleVelocityDegreesPerSecond = Units.radiansToDegrees(rotationSpeedRadPerSecond); // Set the rotation speed
    }
}
