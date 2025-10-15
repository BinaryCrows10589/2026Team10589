package binarycrows.robot.SeasonCode.Subsystems.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModuleIO {


    /**
     * Sets the desired voltage of the module
     * @param desiredVoltage Double: The desired module voltage
     */
    public default void setDesiredModuleDriveVoltage(double desiredVoltage) {

    }

    /**
     * Sets the desired RPM of the module
     * @param desiredRPM Double: The desired RPM for the module
     */
    public default void setDesiredModuleVelocityRPM(double desiredRPM) {}

    /**
     * Sets the desired angle of the modules wheel
     * @param desiredModuleAngle Rotation2d: The desired angle of the modules wheel
     */
    public default void setDesiredModuleAngle(Rotation2d desiredModuleAngle) {}
}
