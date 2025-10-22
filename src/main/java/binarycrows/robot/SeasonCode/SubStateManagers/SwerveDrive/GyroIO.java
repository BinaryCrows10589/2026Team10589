package binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    
    public default void resetAngle(Rotation2d newZero) {}

}
