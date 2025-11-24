package binarycrows.robot.CrowMotion.UserSide;

/**
 * Represents an autonomous point for CrowMotion pathing, with optional mirroring 
 * and alliance-specific positional adjustments (fudging).
 * <p>
 * All distance units are in meters.
 */
public class CMAutonPoint {
    private double x;
    private double y;
    private boolean shouldMirror = false;
    private double blueXFudge = 0;
    private double blueYFudge = 0;
    private double redXFudge = 0;
    private double redYFudge = 0;

    /**
     * Constructs a CMAutonPoint with full specification.
     *
     * @param x             the base x-coordinate in meters
     * @param y             the base y-coordinate in meters
     * @param blueXFudge    the x offset for blue alliance in meters
     * @param blueYFudge    the y offset for blue alliance in meters
     * @param redXFudge     the x offset for red alliance in meters
     * @param redYFudge     the y offset for red alliance in meters
     * @param shouldMirror  whether to mirror the y-coordinate across the field
     */
    public CMAutonPoint(double x, double y, double blueXFudge, double blueYFudge, double redXFudge, double redYFudge, boolean shouldMirror) {
        this.x = x;
        this.y = y;
        this.blueXFudge = blueXFudge;
        this.blueYFudge = blueYFudge;
        this.redXFudge = redXFudge;
        this.redYFudge = redYFudge;
        this.shouldMirror = shouldMirror;
    }

    /**
     * Constructs a CMAutonPoint with mirror setting determined from configuration.
     *
     * @param x             the base x-coordinate in meters
     * @param y             the base y-coordinate in meters
     * @param blueXFudge    the x offset for blue alliance in meters
     * @param blueYFudge    the y offset for blue alliance in meters
     * @param redXFudge     the x offset for red alliance in meters
     * @param redYFudge     the y offset for red alliance in meters
     */
    public CMAutonPoint(double x, double y, double blueXFudge, double blueYFudge, double redXFudge, double redYFudge) {
        this(x, y, blueXFudge, blueYFudge, redXFudge, redYFudge, CMConfig.shouldMirror());
    }

    /**
     * Constructs a CMAutonPoint with no alliance-specific fudging and 
     * mirror setting determined from configuration.
     *
     * @param x the base x-coordinate in meters
     * @param y the base y-coordinate in meters
     */
    public CMAutonPoint(double x, double y) {
        this(x, y, 0, 0, 0, 0, CMConfig.shouldMirror());
    }

    /**
     * Constructs a CMAutonPoint with no alliance-specific fudging and 
     * mirror setting determined from configuration.
     *
     * @param x the base x-coordinate in meters
     * @param y the base y-coordinate in meters
     * @param shouldMirror whether to mirror the y-coordinate across the field
     */
    public CMAutonPoint(double x, double y, boolean shouldMirror) {
        this(x, y, 0, 0, 0, 0, shouldMirror);
    }


    /**
     * Gets the x-coordinate adjusted by alliance-specific fudge values.
     *
     * @return the adjusted x-coordinate in meters
     */
    public double getX() {
        return this.x + (CMConfig.isBlueAlliance() ? blueXFudge : redXFudge);
    }

    /**
     * Gets the y-coordinate, optionally mirrored and adjusted by alliance-specific fudge values.
     *
     * @return the adjusted y-coordinate in meters
     */
    public double getY() {
        return (this.shouldMirror ? (CMConfig.getFieldWidthMeters() - this.y) : this.y) + 
            (CMConfig.isBlueAlliance() ? blueYFudge : redYFudge);
    }

    @Override
    public String toString() {
        return String.format("CMAutonPoint[x=%.3f, y=%.3f, mirrored=%b]", x, y, shouldMirror);
    }
}
