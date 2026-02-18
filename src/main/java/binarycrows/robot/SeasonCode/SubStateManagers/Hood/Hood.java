package binarycrows.robot.SeasonCode.SubStateManagers.Hood;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import binarycrows.robot.Robot;
import binarycrows.robot.SeasonCode.Constants.HoodConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodIO.HoodOutputs;
import binarycrows.robot.Utils.Tuning.RuntimeTunableValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class Hood {
    

    private final HoodIO hoodIO;

    private double externalControlValue;

    private LoggedMechanism2d hoodMech2d;
    private LoggedMechanismRoot2d hoodMechRoot;
    private LoggedMechanismLigament2d hoodMechLigament;

    private RuntimeTunableValue correctionVelocityTunerRadPerSec;
    private RuntimeTunableValue correctionZoneTunerRad;
    private RuntimeTunableValue decelerationBufferTunerRad;
    private RuntimeTunableValue startingVelocityTunerRadPerSec;
    private RuntimeTunableValue correctionFactorTuningDeltaThresholdTunerRad;
    private RuntimeTunableValue maxTurretVelocityTunerRadPerSec;
    private RuntimeTunableValue minTurretVelocityTunerRadPerSec;
    private RuntimeTunableValue maxAccelerationPerFrameTunerRadPerSecPerSec;
    private RuntimeTunableValue maxDecelerationPerFrameTunerRadPerSecPerSec;

    public enum HoodControlType {
        PID,
        VOLTAGE,
        LIKE_TURRET
    }

    private HoodControlType hoodControlType;

    public Hood(HoodIO hoodIO) {
        this.hoodIO = hoodIO;

        hoodMech2d = new LoggedMechanism2d(1, 1);
        hoodMechRoot = hoodMech2d.getRoot("hood", 0.5, 0.5);
        hoodMechLigament = new LoggedMechanismLigament2d("hoodLigament", 0.5, 0);
        hoodMechRoot.append(hoodMechLigament);

        correctionVelocityTunerRadPerSec = new RuntimeTunableValue("HoodTuning/correctionVelocityTunerRadPerSec", HoodConstants.correctionVelocityRadPerSec);
        correctionZoneTunerRad = new RuntimeTunableValue("HoodTuning/correctionZoneTunerRad", HoodConstants.correctionZone.getRadians());
        decelerationBufferTunerRad = new RuntimeTunableValue("HoodTuning/decelerationBufferTunerRad", HoodConstants.decelerationBufferRad);
        startingVelocityTunerRadPerSec = new RuntimeTunableValue("HoodTuning/startingVelocityTunerRadPerSec", HoodConstants.startingVelocityRadPerSec);
        correctionFactorTuningDeltaThresholdTunerRad = new RuntimeTunableValue("HoodTuning/correctionFactorTuningDeltaThresholdTunerRad", HoodConstants.correctionFactorTuningDeltaThresholdRad);
        maxTurretVelocityTunerRadPerSec = new RuntimeTunableValue("HoodTuning/maxTurretVelocityTunerRadPerSec", HoodConstants.maxHoodVelocityRadPerSec);
        minTurretVelocityTunerRadPerSec = new RuntimeTunableValue("HoodTuning/minTurretVelocityTunerRadPerSec", HoodConstants.minHoodVelocityRadPerSec);
        maxAccelerationPerFrameTunerRadPerSecPerSec = new RuntimeTunableValue("HoodTuning/maxAccelerationPerFrameTunerRadPerSecPerSec", HoodConstants.maxAccelerationPerFrameRadPerSecPerSec);
        maxDecelerationPerFrameTunerRadPerSecPerSec = new RuntimeTunableValue("HoodTuning/maxDecelerationPerFrameTunerRadPerSecPerSec", HoodConstants.maxDecelerationPerFrameRadPerSecPerSec);
    }

    private void updateTunables() {
        if (MetaConstants.inProduction) return;

        HoodConstants.correctionVelocityRadPerSec = (double) correctionVelocityTunerRadPerSec.getValue();
        HoodConstants.correctionZone = Rotation2d.fromRadians((double) correctionZoneTunerRad.getValue());
        HoodConstants.decelerationBufferRad = (double) decelerationBufferTunerRad.getValue();
        HoodConstants.startingVelocityRadPerSec = (double) startingVelocityTunerRadPerSec.getValue();
        HoodConstants.correctionFactorTuningDeltaThresholdRad = (double) correctionFactorTuningDeltaThresholdTunerRad.getValue();
        HoodConstants.maxHoodVelocityRadPerSec = (double) maxTurretVelocityTunerRadPerSec.getValue();
        HoodConstants.minHoodVelocityRadPerSec = (double) minTurretVelocityTunerRadPerSec.getValue();
        HoodConstants.maxAccelerationPerFrameRadPerSecPerSec = (double) maxAccelerationPerFrameTunerRadPerSecPerSec.getValue();
        HoodConstants.maxDecelerationPerFrameRadPerSecPerSec = (double) maxDecelerationPerFrameTunerRadPerSecPerSec.getValue();
    }

    public void update() {
        updateTunables();

        hoodIO.update();

        if (hoodControlType != null) {

            switch (hoodControlType) {
                case PID:
                    hoodIO.setTargetPosition(Rotation2d.fromRadians(externalControlValue));
                    break;
                case VOLTAGE:
                    hoodIO.setRotorVoltage(externalControlValue);
                    break;
                case LIKE_TURRET:

                    updateHoodControlLikeTurret();
                    break;
            }
        }
        hoodMechLigament.setAngle(hoodIO.getOutputs().hoodRotation);
        Logger.recordOutput("HoodMech2d", hoodMech2d);
    }

    public void setPIDTarget(Rotation2d position) {
        externalControlValue = position.getRadians();
        hoodControlType = HoodControlType.PID;
    }

    public void setVoltage(double voltage) {
        externalControlValue = voltage;
        hoodControlType = HoodControlType.VOLTAGE;
    }

    public void setTargetLikeTurret(Rotation2d position) {
        externalControlValue = position.getRadians();
        hoodControlType = HoodControlType.LIKE_TURRET;
    }

    private double correctionFactor = 1;

    private double estimatedTravelLastFrame = 0;

    private Rotation2d hoodPositionLastFrame = Rotation2d.kZero;

    public void updateHoodControlLikeTurret() {
        double desiredVelocityRadPerSec;

        HoodOutputs outputs = hoodIO.getOutputs();

        Rotation2d targetHoodPosition = Rotation2d.fromRadians(externalControlValue);

        double delta = targetHoodPosition.getRadians() - outputs.hoodRotation.getRadians();

        Logger.recordOutput("Hood/Control/Delta", delta);

        if (Math.abs(delta) < HoodConstants.correctionZone.getRadians()) { // Within correction zone
            desiredVelocityRadPerSec = HoodConstants.correctionVelocityRadPerSec * delta / HoodConstants.correctionZone.getRadians();

        } else {

            double correctionZoneBorder = targetHoodPosition.getRadians() + HoodConstants.correctionZone.getRadians() * Math.signum(delta);

            delta = correctionZoneBorder - outputs.hoodRotation.getRadians();

            Logger.recordOutput("Hood/Control/DeltaBeforeCorrectionZone", delta);

            double hoodVelocityDirection = Math.signum(outputs.hoodRotationalVelocityRadPerSec);

            if (hoodVelocityDirection == 0) hoodVelocityDirection = Math.signum(delta);

            double maxDesiredVelocity = outputs.hoodRotationalVelocityRadPerSec + HoodConstants.maxAccelerationPerFrameRadPerSecPerSec * hoodVelocityDirection * Robot.averageFrameTime;
            double minDesiredVelocity = outputs.hoodRotationalVelocityRadPerSec - HoodConstants.maxDecelerationPerFrameRadPerSecPerSec * hoodVelocityDirection * Robot.averageFrameTime;
            

            double distanceToStartDecelerating = (outputs.hoodRotationalVelocityRadPerSec * outputs.hoodRotationalVelocityRadPerSec
                        - HoodConstants.correctionVelocityRadPerSec * HoodConstants.correctionVelocityRadPerSec)
                        / (2 * HoodConstants.maxDecelerationPerFrameRadPerSecPerSec) + HoodConstants.decelerationBufferRad;
            boolean forceDecelerate = Math.abs(delta) < distanceToStartDecelerating;

            double targetVelocity = Math.signum(delta) * Math.abs(forceDecelerate ? minDesiredVelocity : maxDesiredVelocity);//delta / Robot.averageFrameTime;  

            Logger.recordOutput("Hood/Control/MaxDesiredVelocity", maxDesiredVelocity);
            Logger.recordOutput("Hood/Control/MinDesiredVelocity", minDesiredVelocity);

            Logger.recordOutput("Hood/Control/RawTargetVelocity", targetVelocity);

            if (targetVelocity > maxDesiredVelocity || targetVelocity < minDesiredVelocity) { // If we are not between max and min...
                // If the difference between us and the max is greater than the difference between us and the min, use the min, otherwise use the max
                targetVelocity = (Math.abs(maxDesiredVelocity - targetVelocity) > Math.abs(minDesiredVelocity - targetVelocity)) ? minDesiredVelocity : maxDesiredVelocity;
            }

            Logger.recordOutput("Hood/Control/CappedTargetVelocity", targetVelocity);
            
            double actualTravelLastFrame = outputs.hoodRotation.minus(hoodPositionLastFrame).getRadians();

            if (Math.abs(actualTravelLastFrame) > HoodConstants.correctionFactorTuningDeltaThresholdRad)
            correctionFactor = estimatedTravelLastFrame / actualTravelLastFrame * .2 + correctionFactor * .8; // Potentially tune the proportions or remove them if not needed
            
            Logger.recordOutput("Hood/Control/CorrectionFactor", correctionFactor);

            if (Math.signum(outputs.hoodRotationalVelocityRadPerSec) == Math.signum(delta)) {
                targetVelocity = Math.signum(targetVelocity) * Math.max(Math.abs(targetVelocity), HoodConstants.minHoodVelocityRadPerSec);
            }

            Logger.recordOutput("Hood/Control/EvenMoreCappedTargetVelocity", targetVelocity);

            estimatedTravelLastFrame = (outputs.hoodRotationalVelocityRadPerSec + targetVelocity) / 2 * Robot.averageFrameTime;
            hoodPositionLastFrame = outputs.hoodRotation;

            desiredVelocityRadPerSec = targetVelocity;

            if (Math.abs(desiredVelocityRadPerSec) < HoodConstants.startingVelocityRadPerSec) {
                desiredVelocityRadPerSec = Math.signum(delta) * HoodConstants.startingVelocityRadPerSec;
            }
            
            //desiredVelocityRadPerSec *= correctionFactor;
        }

        desiredVelocityRadPerSec = Math.signum(desiredVelocityRadPerSec) * Math.min(Math.abs(desiredVelocityRadPerSec), HoodConstants.maxHoodVelocityRadPerSec);
        
        hoodIO.setRotorVoltage(HoodConstants.velocityToVoltageLerpTable.get(desiredVelocityRadPerSec));

    }
}
