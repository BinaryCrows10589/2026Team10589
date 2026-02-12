package binarycrows.robot.SeasonCode.Constants;

public final class CANIDs {
    public final class CANivore {
        public static final int frontLeftDriveMotor = 5;
        public static final int frontRightDriveMotor = 6;
        public static final int backLeftDriveMotor = 7;
        public static final int backRightDriveMotor = 8;

        public static final int frontLeftTurnMotor = 9;
        public static final int frontRightTurnMotor = 10;
        public static final int backLeftTurnMotor = 11;
        public static final int backRightTurnMotor = 12;

        public static final int frontLeftTurnEncoderCANID = 13;
        public static final int frontRightTurnEncoderCANID = 14;
        public static final int backLeftTurnEncoderCANID = 15;
        public static final int backRightTurnEncoderCANID = 16;
        public static final int gyro = 17;

    }

    public final class RIO {
        public static final int leftIntakeRoller = 0;
        public static final int rightIntakeRoller = 1;

        public static final int turretMotor = 2;
        public static final int turretEncoder = 3;

        public static final int hoodMotor = 4;
        public static final int hoodEncoder = 5;

        public static final int leftFlywheelMotor = 6;
        public static final int rightFlywheelMotor = 7;

        public static final int leftPivotMotor = 8;
        public static final int rightPivotMotor = 9;
        public static final int pivotEncoder = 10;

        public static final int leftLongitudinalMotor = 11;
        public static final int rightLongitudinalMotor = 12;
        public static final int leftLatitudinalMotor = 13;
        public static final int rightLatitudinalMotor = 14;

        public static final int binEmptyTOFSensor = 15;
        public static final int binFullTOFSensor = 16;
        public static final int outgoingFuelTOFSensor = 17;

        public static final int CANdle = 37;

    }
}
