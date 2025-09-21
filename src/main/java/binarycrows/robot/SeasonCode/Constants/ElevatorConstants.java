package binarycrows.robot.SeasonCode.Constants;

import java.util.AbstractMap;
import java.util.Map;
import java.util.AbstractMap.SimpleEntry;

import binarycrows.robot.SeasonCode.Subsystems.Elevator.ElevatorStateRequest;

public final class ElevatorConstants {
    public static final double defaultElevatorPosition = 0;

    private static SimpleEntry<ElevatorStateRequest, Double> createElevatorPositionEntry(ElevatorStateRequest elevatorPositionKey, double elevatorPositionValue) {
        return new AbstractMap.SimpleEntry<ElevatorStateRequest, Double>(elevatorPositionKey, elevatorPositionValue);
    }

    public static final Map<ElevatorStateRequest, Double> elevatorPositions = Map.ofEntries(
        createElevatorPositionEntry(ElevatorStateRequest.BASEMENT,        .0),

        createElevatorPositionEntry(ElevatorStateRequest.CORAL_L1,        .281),
        createElevatorPositionEntry(ElevatorStateRequest.CORAL_L2,        .366),
        createElevatorPositionEntry(ElevatorStateRequest.CORAL_L3,        .556),
        createElevatorPositionEntry(ElevatorStateRequest.CORAL_L4,        .8545),

        createElevatorPositionEntry(ElevatorStateRequest.FUNNEL,          .0),
        createElevatorPositionEntry(ElevatorStateRequest.ALGAE_GROUND,    .0),
        createElevatorPositionEntry(ElevatorStateRequest.ALGAE_LOW,       .301),
        createElevatorPositionEntry(ElevatorStateRequest.ALGAE_HIGH,      .506),
        createElevatorPositionEntry(ElevatorStateRequest.ALGAE_PROCESSOR, .0),
        createElevatorPositionEntry(ElevatorStateRequest.ALGAE_BARGE,     .85)
    );

    public static final int masterMotorCANID = 24; // Left motor is master
    public static final int slaveMotorCANID = 25; // Right motor is slave
    public static final int encoderCANID = 26;

    public static final double encoderOffset = .022;

    public static final double catchTolerance = 0.1;

    public static final double forwardSoftLimit = .8643;
    public static final double reverseSoftLimit = 0.0;

    public static final double gearRatio = 6.875;

    public static final double maxVoltage = 13;

    public static final double motionMagicCruiseVelocity = 0;
    public static final double motionMagicAcceleration = 0;
    public static final double motionMagicJerk = 0;

    public static final double PPIDValue = 45;
    public static final double IPIDValue = 0;
    public static final double DPIDValue = 0;
    public static final double GPIDValue = 1.35;

    public static final double SPIDValue = 0;
    public static final double VPIDValue = 0;
    public static final double APIDValue = 0;

    public static final boolean isSlaveReversed = false;


}
