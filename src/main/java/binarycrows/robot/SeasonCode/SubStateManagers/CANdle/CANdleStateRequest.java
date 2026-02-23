package binarycrows.robot.SeasonCode.SubStateManagers.CANdle;

public enum CANdleStateRequest {
    RED,
    GREEN,
    BLUE,

    // TODO: acutally make use of these
    SHOOT_OUT_OF_RANGE,
    SHOOT_CLOSE_TO_RANGE,
    SHOOT_IN_RANGE,
    HOPPER_FULL,
    HOPPER_EMPTY
}
