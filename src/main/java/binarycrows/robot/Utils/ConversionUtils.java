package binarycrows.robot.Utils;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ConversionUtils {

    public static double inchesToMeters(double inches) {
        return inches / 39.37;
    }

    public static double feetToMeters(double feet) {
        return feet * 12 / 39.37;
    }

    public static SwerveModuleState optimizeSwerveModuleState(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90){
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }        
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
      }
 
      public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }

         newAngle += (newAngle < lowerBound) ? Math.ceil((lowerBound - newAngle) / 360) * 360 :
                    (newAngle > upperBound) ? Math.ceil((newAngle - upperBound) / 360) * -360 : 0;

        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
      }
    //TODO:(ELIJAH) Indentiaon is wrong compared to the rest of the file
        @SuppressWarnings("unchecked")
        public static <E> HashMap<String, E> fieldsToMap(Field[] fields, Object obj) {
            HashMap<String, E> map = new HashMap<String, E>();
            for (int i = 0; i < fields.length; i++) {
                try {
                    map.put(fields[i].getName(), (E) fields[i].get(obj));
                } catch (IllegalArgumentException e) {
                    e.printStackTrace();
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                }
            }
            return map;
        }

        public static <T, E> T getKeyByValue(HashMap<T, E> map, E value) {
            for (Entry<T, E> entry : map.entrySet()) {
                if (Objects.equals(value, entry.getValue())) {
                    return entry.getKey();
                }
            }
            return null;
        }

    public static boolean getIsInTolerance(double currentValue, double desiredValue, double tolerance) {
        return Math.abs(currentValue - desiredValue) <= tolerance;
    }

    public static boolean getIsInTolerance(Rotation2d currentValue, Rotation2d desiredValue, double toleranceRad) {
        return Math.abs(currentValue.minus(desiredValue).getRadians()) <= toleranceRad;
    }

    public static boolean getIsInTolerance(Pose2d currentValue, Pose2d desiredValue, Pose2d tolerance) {
        return (getIsInTolerance(currentValue.getX(), desiredValue.getX(), tolerance.getX()) &&
            getIsInTolerance(currentValue.getY(), desiredValue.getY(), tolerance.getY()) && 
            getIsInTolerance(currentValue.getRotation(), desiredValue.getRotation(), tolerance.getRotation().getRadians()));
        
    }

    public static double applyDeadband(double rawValue, double deadband) {
        return rawValue * rawValue * Math.signum(rawValue);
        /*if (Math.abs(rawValue) <= deadband) return 0;
        return (rawValue - deadband) / (1 - deadband);*/
    }

    public static boolean[] getMembersInTolerance(Pose2d currentValue, Pose2d desiredValue, Pose2d tolerance) {
        return new boolean[] {
            getIsInTolerance(currentValue.getX(), desiredValue.getX(), tolerance.getX()),
            getIsInTolerance(currentValue.getY(), desiredValue.getY(), tolerance.getY()),
            getIsInTolerance(currentValue.getRotation(), desiredValue.getRotation(), tolerance.getRotation().getRadians())
        };
    }

}
