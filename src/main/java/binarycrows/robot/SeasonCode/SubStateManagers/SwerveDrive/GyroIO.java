package binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {

    public class GyroOutputs {
        public Rotation2d yawAngle = Rotation2d.kZero;
    }

    public default void update() {}
    
    public default void resetAngle(Rotation2d newZero) {}

}
