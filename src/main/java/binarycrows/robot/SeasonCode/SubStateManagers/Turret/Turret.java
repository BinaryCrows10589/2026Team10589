package binarycrows.robot.SeasonCode.SubStateManagers.Turret;


import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import binarycrows.robot.Robot;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.TurretConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretIO.TurretOutputs;
import binarycrows.robot.Utils.ConversionUtils;
import binarycrows.robot.Utils.Tuning.RuntimeTunableValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret {

    private final TurretIO turretIO;

    private Rotation2d targetTurretPosition;

    private LoggedMechanism2d turretMech2d;
    private LoggedMechanismRoot2d turretMechRoot;
    private LoggedMechanismLigament2d turretMechLigament;

    private RuntimeTunableValue correctionVelocityTunerRadPerSec;
    private RuntimeTunableValue correctionZoneTunerRad;
    private RuntimeTunableValue decelerationBufferTunerRad;
    private RuntimeTunableValue startingVelocityTunerRadPerSec;
    private RuntimeTunableValue correctionFactorTuningDeltaThresholdTunerRad;
    private RuntimeTunableValue maxTurretVelocityTunerRadPerSec;
    private RuntimeTunableValue minTurretVelocityTunerRadPerSec;
    private RuntimeTunableValue maxAccelerationPerFrameTunerRadPerSecPerSec;
    private RuntimeTunableValue maxDecelerationPerFrameTunerRadPerSecPerSec;

    
    public Turret(TurretIO turretIO) {
        this.turretIO = turretIO;

        turretMech2d = new LoggedMechanism2d(1, 1);
        turretMechRoot = turretMech2d.getRoot("turret", 0.5, 0.5);
        turretMechLigament = new LoggedMechanismLigament2d("turretLigament", 0.5, 0);
        turretMechRoot.append(turretMechLigament);

        correctionVelocityTunerRadPerSec = new RuntimeTunableValue("correctionVelocityTunerRadPerSec", TurretConstants.correctionVelocityRadPerSec);
        correctionZoneTunerRad = new RuntimeTunableValue("correctionZoneTunerRad", TurretConstants.correctionZone.getRadians());
        decelerationBufferTunerRad = new RuntimeTunableValue("decelerationBufferTunerRad", TurretConstants.decelerationBufferRad);
        startingVelocityTunerRadPerSec = new RuntimeTunableValue("startingVelocityTunerRadPerSec", TurretConstants.startingVelocityRadPerSec);
        correctionFactorTuningDeltaThresholdTunerRad = new RuntimeTunableValue("correctionFactorTuningDeltaThresholdTunerRad", TurretConstants.correctionFactorTuningDeltaThresholdRad);
        maxTurretVelocityTunerRadPerSec = new RuntimeTunableValue("maxTurretVelocityTunerRadPerSec", TurretConstants.maxTurretVelocityRadPerSec);
        minTurretVelocityTunerRadPerSec = new RuntimeTunableValue("minTurretVelocityTunerRadPerSec", TurretConstants.minTurretVelocityRadPerSec);
        maxAccelerationPerFrameTunerRadPerSecPerSec = new RuntimeTunableValue("maxAccelerationPerFrameTunerRadPerSecPerSec", TurretConstants.maxAccelerationPerFrameRadPerSecPerSec);
        maxDecelerationPerFrameTunerRadPerSecPerSec = new RuntimeTunableValue("maxDecelerationPerFrameTunerRadPerSecPerSec", TurretConstants.maxDecelerationPerFrameRadPerSecPerSec);

    }

    private void updateTunables() {
        if (MetaConstants.inProduction) return;

        TurretConstants.correctionVelocityRadPerSec = (double) correctionVelocityTunerRadPerSec.getValue();
        TurretConstants.correctionZone = Rotation2d.fromRadians((double) correctionZoneTunerRad.getValue());
        TurretConstants.decelerationBufferRad = (double) decelerationBufferTunerRad.getValue();
        TurretConstants.startingVelocityRadPerSec = (double) startingVelocityTunerRadPerSec.getValue();
        TurretConstants.correctionFactorTuningDeltaThresholdRad = (double) correctionFactorTuningDeltaThresholdTunerRad.getValue();
        TurretConstants.maxTurretVelocityRadPerSec = (double) maxTurretVelocityTunerRadPerSec.getValue();
        TurretConstants.minTurretVelocityRadPerSec = (double) minTurretVelocityTunerRadPerSec.getValue();
        TurretConstants.maxAccelerationPerFrameRadPerSecPerSec = (double) maxAccelerationPerFrameTunerRadPerSecPerSec.getValue();
        TurretConstants.maxDecelerationPerFrameRadPerSecPerSec = (double) maxDecelerationPerFrameTunerRadPerSecPerSec.getValue();

    }

    public void update() {
        updateTunables();

        turretIO.update();

        if (targetTurretPosition != null) {
            updateTurretControl();
        }
        turretMechLigament.setAngle(turretIO.getOutputs().turretRotation);
        Logger.recordOutput("TurretMech2d", turretMech2d);
    }

    private double correctionFactor = 1;

    private double estimatedTravelLastFrame = 0;

    private Rotation2d turretPositionLastFrame = Rotation2d.kZero;

    // Now add wire management and stuff given 0-360 range target

    public void updateTurretControl() {
        double desiredVelocityRadPerSec;

        TurretOutputs outputs = turretIO.getOutputs();

        double delta = targetTurretPosition.getRadians() - outputs.turretRotation.getRadians();

        Logger.recordOutput("Turret/Control/Delta", delta);

        if (Math.abs(delta) < TurretConstants.correctionZone.getRadians()) { // Within correction zone
            desiredVelocityRadPerSec = TurretConstants.correctionVelocityRadPerSec * delta / TurretConstants.correctionZone.getRadians();

        } else {

            

            double correctionZoneBorder = targetTurretPosition.getRadians() + TurretConstants.correctionZone.getRadians() * Math.signum(delta);

            delta = correctionZoneBorder - outputs.turretRotation.getRadians();

            Logger.recordOutput("Turret/Control/DeltaBeforeCorrectionZone", delta);

            double turretVelocityDirection = Math.signum(outputs.turretRotationalVelocityRadPerSec);

            if (turretVelocityDirection == 0) turretVelocityDirection = Math.signum(delta);

            double maxDesiredVelocity = outputs.turretRotationalVelocityRadPerSec + TurretConstants.maxAccelerationPerFrameRadPerSecPerSec * turretVelocityDirection * Robot.averageFrameTime;
            double minDesiredVelocity = outputs.turretRotationalVelocityRadPerSec - TurretConstants.maxDecelerationPerFrameRadPerSecPerSec * turretVelocityDirection * Robot.averageFrameTime;
            

            double distanceToStartDecelerating = (outputs.turretRotationalVelocityRadPerSec * outputs.turretRotationalVelocityRadPerSec
                        - TurretConstants.correctionVelocityRadPerSec * TurretConstants.correctionVelocityRadPerSec)
                        / (2 * TurretConstants.maxDecelerationPerFrameRadPerSecPerSec) + TurretConstants.decelerationBufferRad;
            boolean forceDecelerate = Math.abs(delta) < distanceToStartDecelerating;

            double targetVelocity = Math.signum(delta) * Math.abs(forceDecelerate ? minDesiredVelocity : maxDesiredVelocity);//delta / Robot.averageFrameTime;  

            Logger.recordOutput("Turret/Control/MaxDesiredVelocity", maxDesiredVelocity);
            Logger.recordOutput("Turret/Control/MinDesiredVelocity", minDesiredVelocity);

            Logger.recordOutput("Turret/Control/RawTargetVelocity", targetVelocity);

            if (targetVelocity > maxDesiredVelocity || targetVelocity < minDesiredVelocity) { // If we are not between max and min...
                // If the difference between us and the max is greater than the difference between us and the min, use the min, otherwise use the max
                targetVelocity = (Math.abs(maxDesiredVelocity - targetVelocity) > Math.abs(minDesiredVelocity - targetVelocity)) ? minDesiredVelocity : maxDesiredVelocity;
            }

            Logger.recordOutput("Turret/Control/CappedTargetVelocity", targetVelocity);
            
            double actualTravelLastFrame = outputs.turretRotation.minus(turretPositionLastFrame).getRadians();

            if (Math.abs(actualTravelLastFrame) > TurretConstants.correctionFactorTuningDeltaThresholdRad)
            correctionFactor = estimatedTravelLastFrame / actualTravelLastFrame * .2 + correctionFactor * .8; // Potentially tune the proportions or remove them if not needed
            
            Logger.recordOutput("Turret/Control/CorrectionFactor", correctionFactor);

            if (Math.signum(outputs.turretRotationalVelocityRadPerSec) == Math.signum(delta)) {
                targetVelocity = Math.signum(targetVelocity) * Math.max(Math.abs(targetVelocity), TurretConstants.minTurretVelocityRadPerSec);
            }

            Logger.recordOutput("Turret/Control/EvenMoreCappedTargetVelocity", targetVelocity);

            estimatedTravelLastFrame = (outputs.turretRotationalVelocityRadPerSec + targetVelocity) / 2 * Robot.averageFrameTime;
            turretPositionLastFrame = outputs.turretRotation;

            desiredVelocityRadPerSec = targetVelocity;

            if (Math.abs(desiredVelocityRadPerSec) < TurretConstants.startingVelocityRadPerSec) {
                desiredVelocityRadPerSec = Math.signum(delta) * TurretConstants.startingVelocityRadPerSec;
            }
            
            desiredVelocityRadPerSec *= correctionFactor;
        }

        desiredVelocityRadPerSec = Math.signum(desiredVelocityRadPerSec) * Math.min(Math.abs(desiredVelocityRadPerSec), TurretConstants.maxTurretVelocityRadPerSec);
        

        

        //TODO: The lerp table might be broke unless I am very smart
        turretIO.setRotorVoltage(TurretConstants.velocityToVoltageLerpTable.get(desiredVelocityRadPerSec));

    }

    public void setTargetTurretPosition(Rotation2d targetPosition) {
        targetTurretPosition = targetPosition;
    }

    public void setTurretVoltage(double voltage) {
        turretIO.setRotorVoltage(voltage);
    }

    public void setTurretVelocity(double velocityRadPerSec) {
        turretIO.setRotorVoltage(TurretConstants.velocityToVoltageLerpTable.get(velocityRadPerSec));
    }

    public void setTargetTurretPosition(double targetPosition) {
        targetTurretPosition = Rotation2d.fromDegrees(targetPosition);
    }

    public void resetMotorToAbsolute() {
        turretIO.resetMotorToAbsolute();
    }

}
