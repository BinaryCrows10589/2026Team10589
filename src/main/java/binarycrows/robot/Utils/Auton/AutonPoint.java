package binarycrows.robot.Utils.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutonPoint {
    private Pose2d autonPoint;
    private FudgeFactor fudgeFactor;
    private double attackAngleDegrees;
    //private boolean allowMirroring;
    
    /**
     * Current mirroring assumes mirrored standared mirrored field rather than a rotated or flipped.
     * @param xPointMeters The x coordnet in meters
     * @param yPointMeters The y coordnet in meters
     * @param rotationAngleDegrees The rotation angle in degrees
     * @param fudgeFactor
     */
    public AutonPoint(double xPointMeters, double yPointMeters, double rotationAngleDegrees, FudgeFactor fudgeFactor) {
        this(xPointMeters, yPointMeters, rotationAngleDegrees, 0, fudgeFactor);
    }

    /**
     * Current mirroring assumes assumes mirrored standared mirrored field rather than a rotated or flipped.
     * @param xPointMeters The x coordnet in meters
     * @param yPointMeters The y coordnet in meters
     * @param rotationAngleDegrees The rotation angle in degrees
     * @param attackAngleDegrees the rotation that the robot will have as it enters into this point during an autonomous period
     *      * @param fudgeFactor

     */
    public AutonPoint(double xPointMeters, double yPointMeters, double rotationAngleDegrees, double attackAngleDegrees,  FudgeFactor fudgeFactor) {
        this.autonPoint = new Pose2d(xPointMeters, yPointMeters, Rotation2d.fromDegrees(rotationAngleDegrees));
        this.fudgeFactor = fudgeFactor;
        this.attackAngleDegrees = attackAngleDegrees;
    }

    /**
     * Create a point to represent a coordinate in autonomous or for vision systems.
     * @param pose Pose2d: The x, y and rotaiton value of the point
     * @param fudgeFactor FudgeFactor: The fudge factor of the point
     * @param attackAngleDegrees the rotation that the robot will have as it enters into this point during an autonomous period
     */
    public AutonPoint(Pose2d pose, FudgeFactor fudgeFactor, double attackAngleDegrees) {
        this.autonPoint = pose;
        this.fudgeFactor = fudgeFactor;
        this.attackAngleDegrees = attackAngleDegrees;
    }

    /**
     * Create a point to represent a coordinate in autonomous or for vision systems.
     * @param pose Pose2d: The x, y and rotaiton value of the point
     * @param fudgeFactor
     */
    public AutonPoint(Pose2d pose, FudgeFactor fudgeFactor) {
        this(pose, fudgeFactor, 0);
    }

    /**
     * Create a point to represent a coordinate in autonomous or for vision systems.
     * @param pose Pose2d: The Pose2D representing the x, y and rotation value of the point
     */
    public AutonPoint(Pose2d pose) {
        this(pose, 0);
    }

    /**
     * Create a point to represent a coordinate in autonomous or for vision systems.
     * @param pose Pose2d: The Pose2D representing the x, y and rotation value of the point
     * @param attackAngleDegrees the rotation that the robot will have as it enters into this point during an autonomous period
     */
    public AutonPoint(Pose2d pose, double attackAngleDegrees) {
        this.autonPoint = pose;
        this.fudgeFactor = new FudgeFactor(0, 0, 0);
        this.attackAngleDegrees = attackAngleDegrees;
    }

    /**
     * Create a point to represent a coordinate in autonomous or for vision systems.
     * @param xPointMeters The x coordnet in meters
     * @param yPointMeters The y coordnet in meters
     * @param rotationAngleDegrees The rotation angle in degrees
     * @param attackAngleDegrees the rotation that the robot will have as it enters into this point during an autonomous period
     */
    public AutonPoint(double xPointMeters, double yPointMeters, double rotationAngleDegrees, double attackAngleDegrees) {
        this(xPointMeters, yPointMeters, rotationAngleDegrees, attackAngleDegrees, true);
    }
    /**
     * Create a point to represent a coordinate in autonomous or for vision systems.
     * @param xPointMeters The x coordnet in meters
     * @param yPointMeters The y coordnet in meters
     * @param rotationAngleDegrees The rotation angle in degrees
     */
    public AutonPoint(double xPointMeters, double yPointMeters, double rotationAngleDegrees) {
        this(xPointMeters, yPointMeters, rotationAngleDegrees, 0);
    }

    /**
     * Create a point to represent a coordinate in autonomous or for vision systems.
     * @param xPointMeters The x coordnet in meters
     * @param yPointMeters The y coordnet in meters
     * @param rotationAngleDegrees The rotation angle in degrees
     * @param allowMirroring Should the point be able to mirror
     */
    public AutonPoint(double xPointMeters, double yPointMeters, double rotationAngleDegrees, boolean allowMirroring) {
        this(xPointMeters, yPointMeters, rotationAngleDegrees, 0, allowMirroring);
    }

    /**
     * Create a point to represent a coordinate in autonomous or for vision systems.
     * @param xPointMeters The x coordnet in meters
     * @param yPointMeters The y coordnet in meters
     * @param rotationAngleDegrees The rotation angle in degrees
     * @param attackAngleDegrees the rotation that the robot will have as it enters into this point during an autonomous period
     * @param allowMirroring Should the point be able to mirror
     */
    public AutonPoint(double xPointMeters, double yPointMeters, double rotationAngleDegrees, double attackAngleDegrees, boolean allowMirroring) {
        this.autonPoint = new Pose2d(xPointMeters, yPointMeters, Rotation2d.fromDegrees(rotationAngleDegrees));
        this.autonPoint.getRotation().getDegrees();
        this.fudgeFactor = new FudgeFactor(0, 0, 0);
        //this.allowMirroring = allowMirroring;
        this.attackAngleDegrees = attackAngleDegrees;
    }

    /**
     * Create a point to represent a coordinate in autonomous or for vision systems.
     * @param xPointMeters The x coordnet in meters
     * @param yPointMeters The y coordnet in meters
     * @param rotationAngleDegrees The rotation angle in degrees
     * @param attackAngleDegrees the rotation that the robot will have as it enters into this point during an autonomous period
     * @param allowMirroring Should the point be able to mirror
     */
    public AutonPoint(double xPointMeters, double yPointMeters, double rotationAngleDegrees, double attackAngleDegrees, boolean allowMirroring, FudgeFactor fudgeFactor) {
        this.autonPoint = new Pose2d(xPointMeters, yPointMeters, Rotation2d.fromDegrees(rotationAngleDegrees));
        this.autonPoint.getRotation().getDegrees();
        this.fudgeFactor = fudgeFactor;
        //this.allowMirroring = allowMirroring;
        this.attackAngleDegrees = attackAngleDegrees;
    }


    /**
     * Gets the auton point, auto mirrored depending on alliance.
     * @return Pose2d: The auton point which is mirrored for the current alliance.
     */
    public Pose2d getAutonPoint() { 
        /* 
        if(!RobotModeConstants.isBlueAlliance && allowMirroring) {
            return new Pose2d(this.autonPoint.getX() + fudgeFactor.getRedFudgeFactors().getX(),
                (FieldConstants.kFieldWidthMeters - this.autonPoint.getY()) +
                fudgeFactor.getRedFudgeFactors().getY(),
                Rotation2d.fromDegrees((this.autonPoint.getRotation().getDegrees() +
                this.fudgeFactor.getRedFudgeFactors().getRotation().getDegrees()) * -1));
        }
        */  
        //double yOffset = RobotModeConstants.isBlueAlliance ? 0 : -0.0508;
        return new Pose2d(this.autonPoint.getX() + this.fudgeFactor.getAllianceFudgeFactors().getX(),
            this.autonPoint.getY() + this.fudgeFactor.getAllianceFudgeFactors().getY(),
                Rotation2d.fromDegrees((this.autonPoint.getRotation().getDegrees() +
                this.fudgeFactor.getAllianceFudgeFactors().getRotation().getDegrees())));
        
    }  

    /**
     * Returns an autonomous point as a Pose2d where the rotation represents the angle of attack of the point, which is necessary for WPILib
     * @return Pose2d: The auton point
     */
    public Pose2d getWPILibAutonPoint() {
        //double yOffset = RobotModeConstants.isBlueAlliance ? 0 : -0.0508;
        //Logger.recordOutput("AutonPointYOffset", yOffset);
        return new Pose2d(
            this.autonPoint.getX() + this.fudgeFactor.getAllianceFudgeFactors().getX(),
            this.autonPoint.getY() + this.fudgeFactor.getAllianceFudgeFactors().getY(),
            Rotation2d.fromDegrees((
                this.getAngleOfAttack() + this.fudgeFactor.getAllianceAngleOfAttackFudgeFactor()
            ))
        );
    }

    /**
     * Return the attack angle of the point (defaults to zero if none was provided)
     * @return boolean: The attack angle of the autonomous point (for WPILib trajectories)
     */
    public double getAngleOfAttack() {
        return this.attackAngleDegrees;
    }

    public void setAngleOfAttack(double angleOfAttack) {
        this.attackAngleDegrees = angleOfAttack;
    }

    /**
     * Gets the auton point. mirrored bassed on the argument.
     * @param shouldMirror Boolean: shouldMirror Whether or not the point should be mirrored
     * @return Pose2d: The auton point mirrored based on the argument.
     */
    public Pose2d getAutonPoint(boolean shouldMirror) {
       /*if(shouldMirror) {
            return new Pose2d(this.autonPoint.getX() + fudgeFactor.getAllianceFudgeFactors().getX(),
                (FieldConstants.kFieldWidthMeters - this.autonPoint.getY()) +
                fudgeFactor.getRedFudgeFactors().getY(),
                Rotation2d.fromDegrees((this.autonPoint.getRotation().getDegrees() +
                this.fudgeFactor.getRedFudgeFactors().getRotation().getDegrees()) * -1));
        }*/
        //double yOffset = RobotModeConstants.isBlueAlliance ? 0 : -0.0508;
        return new Pose2d(this.autonPoint.getX() + this.fudgeFactor.getAllianceFudgeFactors().getX(),
            this.autonPoint.getY() + this.fudgeFactor.getAllianceFudgeFactors().getY(),
                Rotation2d.fromDegrees((this.autonPoint.getRotation().getDegrees() +
                this.fudgeFactor.getAllianceFudgeFactors().getRotation().getDegrees())));
    }

}
