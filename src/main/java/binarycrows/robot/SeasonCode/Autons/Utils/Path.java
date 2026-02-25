package binarycrows.robot.SeasonCode.Autons.Utils;

import java.util.Objects;

import binarycrows.robot.CrowMotion.UserSide.CMAutonPoint;
import binarycrows.robot.CrowMotion.UserSide.CMRotation;

public record Path(PathRotation[] pathRotations, CMAutonPoint... pathPoints) {

    public record PathRotation(double angleDegrees, int rotationDirection, double percentOfPathToCompleteBy) {
        public CMRotation createCMRotation(double maxToleranceDegrees) {
            return new CMRotation(angleDegrees, rotationDirection, percentOfPathToCompleteBy, maxToleranceDegrees);
        }
        public CMRotation createCMRotation(double maxToleranceDegrees, double maxRotationVelocityDegrees, double desiredRotationalAccelerationDegrees, double desiredRotationDecelerationDegrees) {
            return new CMRotation(angleDegrees, rotationDirection, percentOfPathToCompleteBy, maxRotationVelocityDegrees, desiredRotationalAccelerationDegrees, desiredRotationDecelerationDegrees, maxToleranceDegrees);
        }
    }

}
