package binarycrows.robot.CrowMotion.Library;

import java.awt.geom.Point2D;
import edu.wpi.first.math.geometry.Translation2d;

public class CMPathPoint {

    private Point2D.Double translationalPoint;
    private double distanceFromStart;

    /**
     * Constructs an instance with the specified translational point, rotation, and
     * event.
     *
     * @param translationalPoint The x and y coordnets of the path at this point
     * @param desiredRotation    The desired rotation angle at this point
     * @param event              The event to triggure at this point
     */
    public CMPathPoint(Point2D.Double translationalPoint, double distenceFromStart) {
        this.translationalPoint = translationalPoint;
        this.distanceFromStart = distenceFromStart;
    }

    public static Translation2d[] point2dToTranslation2D(CMPathPoint[] points) {
        Translation2d[] translations = new Translation2d[points.length];
        for (int i = 0; i < points.length; i++) {
            Point2D.Double point = points[i].getTranslationalPoint();
            translations[i] = new Translation2d(point.x, point.y);
        }
        return translations;
    }


    /**
     * Gets the translational point.
     *
     * @return The x and y coordnets of the path at this point
     */
    public Point2D.Double getTranslationalPoint() {
        return translationalPoint;
    }

    public double getDistanceFromStart() {
        return this.distanceFromStart;
    }

}
