package binarycrows.robot.SeasonCode.Subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

import binarycrows.robot.SeasonCode.Constants.ElevatorConstants;

public interface ElevatorIO {
   
    /**
     * Sets the goal position of the elevator 
     * @param desiredPosition Double: The desired position of the elevator
     */
    public default void setDesiredPosition(double desiredPosition) {}

    public default void incrementDesiredPosition(double increment) {}

    public default void disableElevatorMotors() {}

    public default void updateInputs() {}
}
