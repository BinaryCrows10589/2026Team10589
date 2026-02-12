package binarycrows.robot.SeasonCode.SubStateManagers.Turret;

import binarycrows.robot.Robot;
import binarycrows.robot.SeasonCode.Constants.TurretConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretIO.TurretOutputs;
import binarycrows.robot.Utils.ConversionUtils;
import edu.wpi.first.math.geometry.Rotation2d;

public class Turret {

    private final TurretIO turretIO;

    private Rotation2d targetTurretPosition;
    
    public Turret(TurretIO turretIO) {
        this.turretIO = turretIO;
    }

    public void update() {
        turretIO.update();

        if (targetTurretPosition != null) {
            updateTurretControl();
        }
    }

    private double correctionFactor = 1;

    private double estimatedTravelLastFrame = 0;

    private Rotation2d turretPositionLastFrame = Rotation2d.kZero;

    public void updateTurretControl() { // TODO: Add logs
        double desiredVelocityRadPerSec;

        TurretOutputs outputs = turretIO.getOutputs();

        Rotation2d delta = targetTurretPosition.minus(outputs.turretRotation);

        if (Math.abs(delta.getRadians()) < TurretConstants.correctionZone.getRadians()) { // Within correction zone
            desiredVelocityRadPerSec = TurretConstants.correctionVelocityRadPerSec * delta.getRadians() / TurretConstants.correctionZone.getRadians();

        } else {

            Rotation2d correctionZoneBorder = targetTurretPosition.plus(TurretConstants.correctionZone.times(Math.signum(delta.getRadians())));

            delta = correctionZoneBorder.minus(outputs.turretRotation);


            double turretVelocityDirection = Math.signum(outputs.turretRotationalVelocityRadPerSec);

            double maxDesiredVelocity = outputs.turretRotationalVelocityRadPerSec + TurretConstants.maxAccelerationPerFrameRadPerSecPerSec * turretVelocityDirection * Robot.averageFrameTime;
            double minDesiredVelocity = outputs.turretRotationalVelocityRadPerSec - TurretConstants.maxDecelerationPerFrameRadPerSecPerSec * turretVelocityDirection * Robot.averageFrameTime;
            
            double targetVelocity = delta.getRadians() / Robot.averageFrameTime;

            if (targetVelocity > maxDesiredVelocity) {
                targetVelocity = maxDesiredVelocity;
            } else if (targetVelocity < minDesiredVelocity) {
                targetVelocity = minDesiredVelocity;
            }
            
            double actualTravelLastFrame = outputs.turretRotation.minus(turretPositionLastFrame).getRadians();

            correctionFactor = estimatedTravelLastFrame / actualTravelLastFrame * .2 + correctionFactor * .8; // Potentially tune the proportions or remove them if not needed
            
            targetVelocity = Math.max(targetVelocity, maxDesiredVelocity);
            if (Math.signum(outputs.turretRotationalVelocityRadPerSec) == Math.signum(delta.getRadians())) {
                targetVelocity = Math.min(targetVelocity, minDesiredVelocity);
            }

            estimatedTravelLastFrame = (outputs.turretRotationalVelocityRadPerSec + targetVelocity) / 2 * Robot.averageFrameTime;
            turretPositionLastFrame = outputs.turretRotation;

            desiredVelocityRadPerSec = targetVelocity;
        
        }

        //TODO: The lerp table might be broke unless I am very smart
        turretIO.setRotorVoltage(TurretConstants.velocityToVoltageLerpTable.get(desiredVelocityRadPerSec));

    }

    public void setTargetTurretPosition(Rotation2d targetPosition) {
        targetTurretPosition = targetPosition;
    }

    public void resetMotorToAbsolute() {
        turretIO.resetMotorToAbsolute();
    }

}
