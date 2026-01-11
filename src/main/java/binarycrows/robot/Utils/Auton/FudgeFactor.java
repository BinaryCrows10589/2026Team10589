package binarycrows.robot.Utils.Auton;

import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FudgeFactor {
    private Pose2d blueFudgeFactors;
    private Pose2d redFudgeFactors;
    private double blueAngleOfAttackFudgeFactor;
    private double redAngleOfAttackFudgeFactor;

    /**
     * Creates a fudge factor object for both alliances with different values.
     * @param xFudgeFactorMetersBlue Double: The x axis fudge factor in meters for blue
     * @param yFudgeFactorMetersBlue Double: The y axis fudge factor in meters for blue
     * @param rotationAngleFudgeFactorDegreesBlue Double: The rotaiton fudge factor in degrees for blue
     * @param xFudgeFactorMetersRed Double: The x axis fudge factor in meters for red
     * @param yFudgeFactorMetersRed Double: The y axis fudge factor in meters for red
     * @param rotationAngleFudgeFactorDegreesRed Double: The rotaiton fudge factor in degrees for for red
     */
    public FudgeFactor(
        double xFudgeFactorMetersBlue, double yFudgeFactorMetersBlue, double rotationAngleFudgeFactorDegreesBlue,
        double xFudgeFactorMetersRed, double yFudgeFactorMetersRed, double rotationAngleFudgeFactorDegreesRed
        ) {

        this(
            xFudgeFactorMetersBlue, yFudgeFactorMetersBlue, 0, rotationAngleFudgeFactorDegreesBlue,
            xFudgeFactorMetersRed, yFudgeFactorMetersRed, 0, rotationAngleFudgeFactorDegreesRed
        );
    }

    /**
     * Creates a fudge factor object for both alliances with different values.
     * @param xFudgeFactorMetersBlue Double: The x axis fudge factor in meters for blue
     * @param yFudgeFactorMetersBlue Double: The y axis fudge factor in meters for blue
     * @param attackAngleFudgeFactorDegreesBlue Double: The angle of attack fudge factor in degrees for the blue alliance
     * @param rotationAngleFudgeFactorDegreesBlue Double: The rotaiton fudge factor in degrees for blue
     * @param xFudgeFactorMetersRed Double: The x axis fudge factor in meters for red
     * @param yFudgeFactorMetersRed Double: The y axis fudge factor in meters for red
     * @param attackAngleFudgeFactorDegreesRed Double: The angle of attack fudge factor in degrees for the red alliance
     * @param rotationAngleFudgeFactorDegreesRed Double: The rotaiton fudge factor in degrees for for red
     */
    public FudgeFactor(double xFudgeFactorMetersBlue, double yFudgeFactorMetersBlue, double attackAngleFudgeFactorDegreesBlue, double rotationAngleFudgeFactorDegreesBlue,
        double xFudgeFactorMetersRed, double yFudgeFactorMetersRed, double attackAngleFudgeFactorDegreesRed, double rotationAngleFudgeFactorDegreesRed) {

        this.blueFudgeFactors = new Pose2d(xFudgeFactorMetersBlue, yFudgeFactorMetersBlue, Rotation2d.fromDegrees(rotationAngleFudgeFactorDegreesBlue));
        this.redFudgeFactors = new Pose2d(xFudgeFactorMetersRed, yFudgeFactorMetersRed, Rotation2d.fromDegrees(rotationAngleFudgeFactorDegreesRed));
        this.blueAngleOfAttackFudgeFactor = attackAngleFudgeFactorDegreesBlue;
        this.redAngleOfAttackFudgeFactor = attackAngleFudgeFactorDegreesRed;
    }

    /**
     * Creates a fudge factor object for both alliances with the same values.
     * @param xFudgeFactorMeters Double: The x axis fudge factor in meters for both alliances
     * @param yFudgeFactorMeters Double: The y axis fudge factor in meters for both alliances
     * @param rotationAngleFudgeFactorDegrees Double: The rotation fudge factor in degrees for both alliances
     */
    public FudgeFactor(double xFudgeFactorMeters, double yFudgeFactorMeters, double rotationAngleFudgeFactorDegrees) {

        this(
            xFudgeFactorMeters, yFudgeFactorMeters, 0, rotationAngleFudgeFactorDegrees);
    }

    /**
     * Creates a fudge factor object for both alliances with the same values.
     * @param xFudgeFactorMeters Double: The x axis fudge factor in meters for both alliances
     * @param yFudgeFactorMeters Double: The y axis fudge factor in meters for both alliances
     * @param rotationAngleFudgeFactorDegrees Double: The rotation fudge factor in degrees for both alliances
     * @param attackAngleFudgeFactorDegrees Double: The angle of attack fudge factor in degrees for both alliances
     */
    public FudgeFactor(double xFudgeFactorMeters, double yFudgeFactorMeters, double rotationAngleFudgeFactorDegrees, double attackAngleFudgeFactorDegrees) {
        this(
            xFudgeFactorMeters, yFudgeFactorMeters, 0, rotationAngleFudgeFactorDegrees,
            xFudgeFactorMeters, yFudgeFactorMeters, 0, rotationAngleFudgeFactorDegrees
        );
    }

    /**
     * Gets the fudge factors for the blue alliance
     * @return Pose2d: The blue alliance fudge factors
     */
    public Pose2d getBlueFudgeFactors() {

        return this.blueFudgeFactors;
    }

    /**
     * Gets the fudge factors for the red alliance
     * @return Pose2d: The red alliance fudge factors
     */
    public Pose2d getRedFudgeFactors() {
        return this.redFudgeFactors;
    }

    /**
     * Gets the fudge factor for the current alliance by referencing the value of RobotModeConstants.isBlueAlliance
     * @return The correct alliance's fudge factors
     */
    public Pose2d getAllianceFudgeFactors() {
        return MetaConstants.isBlueAlliance ? getBlueFudgeFactors() : getRedFudgeFactors();
    }

    /**
     * Gets the angle of attack fudge factor for the blue alliance
     * @return Double: The blue alliance angle of attack fudge factor
     */
    public double getBlueAngleOfAttackFudgeFactor() {
        return this.blueAngleOfAttackFudgeFactor;
    }

    /**
     * Gets the angle of attack fudge factor for the red alliance
     * @return Double: The red alliance angle of attack fudge factor
     */
    public double getRedAngleOfAttackFudgeFactor() {
        return this.redAngleOfAttackFudgeFactor;
    }

    /**
     * Gets the angle of attack fudge factor for the current alliance by referencing the value of RobotModeConstants.isBlueAlliance
     * @return Double: The correct alliance angle of attack fudge factor
     */
    public double getAllianceAngleOfAttackFudgeFactor() {
        return MetaConstants.isBlueAlliance ? this.getBlueAngleOfAttackFudgeFactor() : this.getRedAngleOfAttackFudgeFactor();
    }

}
