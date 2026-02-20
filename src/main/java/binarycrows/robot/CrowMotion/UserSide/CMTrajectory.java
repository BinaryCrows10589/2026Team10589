package binarycrows.robot.CrowMotion.UserSide;

import java.awt.geom.Point2D;
import java.util.concurrent.CompletableFuture;

import org.littletonrobotics.junction.Logger;

import binarycrows.robot.CrowMotion.Library.CMPathGenResult;
import binarycrows.robot.CrowMotion.Library.CMPathGenerator;
import binarycrows.robot.CrowMotion.Library.CMPathPoint;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;

public class CMTrajectory {
    private String pathName;
    private double drivebaseCircumference;
    private double maxModuleVelocity;
    private double maxDesiredTranslationalVelocity;
    private double desiredTranslationalAcceleration;
    private double desiredTranslationalDecceleration;
    private TrajectoryPriority trajectoryPriority;
    private double endVelocity;
    private double minStartVelocity;
    private double distanceAtEndVelocity;
    private boolean shouldStopAtEnd;
    private double[] positionTolerance;
    private double lookAheadMultiplier;
    private double rotationSettleTime;
    private double maxTime;

    private CompletableFuture<CMPathGenResult> futurePath;
    private CMPathGenResult pathGenResult = null;
    private CMPathPoint[] path = null;
    private CMRotation[] rotationDeadlines = null;
    private CMEvent[] events = null;
    private double[] endRobotState;

    private long endTime = -1;
    private long rotationSettleEndTime = -1;
    private long frameStartTime = -1;

    private boolean isComplete = false;
    private boolean preventTranslation = false;

    private double averageFrameTime = MetaConstants.loopPeriodSeconds;

    private int currentMinPointIndex = 1;
    private int goalPointIndex = 0;
    private CMPathPoint lastMinPointPathPoint = null;
    private CMPathPoint currentMinPointPathPoint = null;
    private CMPathPoint endPoint = null;
    private Point2D.Double lastMinPoint = null;
    private Point2D.Double currentMinPoint = null;
    private Point2D.Double nextMinPoint = null;

    private double lastDesiredVelocity = -1;
    private double velocityDelta = 0;
    private double initialDistanceToEnd = 0;
    private boolean shouldDecelerate = false;
    private boolean firstDecelerationFrame = true;

    private int lastRotationDeadlineIndex = -1;
    private int currentRotationDeadlineIndex = 0;
    private CMRotation rotationDeadline = null;

    private double desiredRotationDegrees = 0;
    private double maxRotationVelocityDegrees = 0;
    private double desiredRotationalAccelerationDegrees = 0;
    private double desiredRotatioanalDecelerationDegrees = 0;
    private int rotationDirection = 0;

    private double rotationCorrectionRangeDegrees = 0;
    private double maxRotationCorrectionVelocityDegrees = 0;
    private double minRotationVelocityToMove = 0;
    private double maxRotationToleranceDegrees = 0;
    private double decelerationBufferDegrees = 0;
    private double rotationDeadlineCompletePercent = 0;

    private double lastDesiredRotationalVelocity = 0;
    private boolean firstRotationDecelerationFrame = true;
    private boolean enteredCorrectionRange = false;
    private boolean shouldDecelerateRotation = false;
    private double initalRotationalVelocity = 0;
    private double initialDegreesToEnd = 0;

    private int lastTriggeredEventIndex = -1;

    public static enum TrajectoryPriority {
        PREFER_ROTATION,
        PREFER_TRANSLATION,
        SPLIT_PROPORTIONALLY
    }

     /**
     * Creates a new {@code CMTranslationConfig} to define the translational motion
     * settings for a trajectory.
     *
     * <p>Each parameter directly affects how the robot follows and finishes the
     * path. These values should be tuned based on whether the trajectory is
     * stopping or non-stopping, the robot’s drivetrain limits, and the desired
     * balance between accuracy and speed.</p>
     ** @param pathName
     *        The name of the path. Used in logging.
     *
     * @param controlPoints
     *        Array of {@link CMAutonPoint} control points defining the Path.
     *        <ul>
     *          <li>If given 1 point, the path is a line from the robot to the point.</li>
     *          <li>If given 2 points, the path is a line from point 1 to point 2.</li>
     *          <li>If given 3 or more points, the path is a curve generated from the points.</li>
     *        </ul>
     *        <strong>Note:</strong> The Bezier curve algorithm only guarantees the 1st and last point
     *        are on the curve; others are control points. This is how all Bezier curve generation works,
     *        including PathPlanner, Choreo, and WPILib Trajectories.
     *
     * @param rotations
     *        Array of {@link CMRotation} objects specifying desired rotation
     *        deadlines (target angles, velocity profiles, and deadlines) along the trajectory.
     *
     * @param events
     *        Array of {@link CMEvent} triggers scheduled to occur at specific
     *        completion percentages of the trajectory.
     *
     * @param pointsPerMeter
     *        Resolution of the generated path, measured as number of interpolated
     *        path points per meter of travel. Higher values yield smoother paths but
     *        heavier computations.
     *        <ul>
     *          <li>Suggested start: 1.5 (Works well; if you want smoother, try 3)</li>
     *          <li>Lower bound: 1</li>
     *          <li>Upper bound: Infinity (anything above 3 is usually pointless)</li>
     *        </ul>
     * @param lookAheadMultiplier
     *        Multiplier used to determine which point along the path the robot should
     *        aim for.
     *        <ul>
     *          <li>Suggested value: 10</li>
     *          <li>Reasonable range: 7–10</li>
     *        </ul>
     *        Lower values improve path-following accuracy but increase side-to-side
     *        oscillation. Higher values reduce oscillation but cause the robot to
     *        cut corners more aggressively.
     * @param trajectoryPriority
     *        How to prioritize velocity when translational and rotational demands
     *        exceed the drivetrain’s capacity:
     *        <ul>
     *          <li>{@code PREFER_ROTATION}, Lowers translatinal speed however much is needed to allow for the rotation speed</li>
     *          <li>{@code PREFER_TRANSLATION}, Lowers rotation speed however much is needed to allow for the translational speed</li>
     *          <li>{@code SPLIT_PROPORTIONALLY}, Lowers both in proportion until the motion is possible</li>
     *        </ul>
     * @param maxDesiredTranslationalVelocity
     *        The maximum translational velocity the robot is allowed to reach while
     *        following the trajectory.
     *        <ul>
     *          <li>Suggested start value: 3.5 m/s</li>
     *          <li>Lower bound: &gt; 0</li>
     *          <li>Upper bound: robot’s maximum safe velocity</li>
     *        </ul>
     * @param desiredTranslationalAcceleration
     *        The maximum translational acceleration allowed while following the
     *        path. Controls how quickly the robot speeds up.
     *        <ul>
     *          <li>Suggested start: 3.5</li>
     *          <li>Lower bound: &gt; 0</li>
     *          <li>Upper bound: Robot’s maximum safe acceleration</li>
     *        </ul>
     *        <strong>Note:</strong> If odometry drift is occurring, decreasing
     *        acceleration will help prevent wheel slippage.
     *        
     * @param desiredTranslationalDecceleration
     *        The deceleration rate used only at the very end of the trajectory to
     *        smoothly reduce the robot’s velocity down to the specified end
     *        velocity by the time it reaches {@code distanceAtendVelocity}.
     *        <ul>
     *          <li>Suggested start: 3.5</li>
     *          <li>Upper bound: Robot’s maximum safe acceleration</li>
     *        </ul>
     *        <strong>Note:</strong> If odometry drift is occurring, decreasing
     *        deceleration will help prevent wheel slippage.
     *        <br>
     * @param minStartVelocity
     *        The initial velocity assigned to the robot when beginning the
     *        trajectory. Prevents excessively slow starts.
     *        <ul>
     *          <li>Suggested start: <b>2.0 m/s</b></li>
     *          <li>Lower bound: &gt; 0</li>
     *        </ul>
     *        This allows the robot to accelerate very quickly at the start, then
     *        transition to smoother acceleration to save time while limiting forces.
     *        <br>
     * @param shouldStopAtEnd
     *        Whether the robot’s velocity should be forced to 0 at the end of the
     *        trajectory.
     *        <ul>
     *          <li>true: Robot stops fully at the end.</li>
     *          <li>false: Robot continues moving, allowing a smooth transition
     *          to another trajectory or driver control.</li>
     *        </ul>
     *        <br>
     * @param endVelocity
     *        The velocity that the robot should be traveling at
     *        while it travels the distance specified by {@code distanceAtendVelocity}.
     *        <ul>
     *          <li>Stopping trajectories: Suggested ~= 0.08 m/s, or the maximum
     *          speed the robot can stop at “instantly."</li>
     *          <li>Non-stopping trajectories: Should be any value ≤
     *          {@code maxDesiredTranslationVelocityMeters}. Allows smooth pass-off to
     *          the next trajectory or driver control.</li>
     *        </ul>
     *        For stopping: decrease to increase accuracy, increase to save time.
     *        <br>
     * @param distanceAtEndVelocity
     *        The distance from the final target point at which the robot should
     *        already be traveling at {@code endVelocity}.
     *        <ul>
     *          <li>Stopping trajectories: Suggested ~= 0.05 m</li>
     *          <li>Non-stopping trajectories: Can be 0</li>
     *        </ul>
     *        Increase this distance to improve accuracy, decrease to save time.
     *        <br>
     * @param positionTolerance
     *        The positional tolerance that determines when the trajectory is
     *        considered complete.
     *        <ul>
     *          <li>Stopping trajectories: Suggested ~= ±0.01 m. Smaller values
     *          increase accuracy.</li>
     *          <li>Non-stopping trajectories: Suggested ≥ ±0.5 m. Looser
     *          tolerance allows smooth transitions to the next trajectory or driver
     *          control.</li>
     *        </ul>
     *        <br>
     
     * @param rotationSettleTime
     *        Time (s) the robot must remain within rotational tolerance before
     *        the trajectory is considered complete. Prevents overshoot from
     *        falsely ending the trajectory.
     *        <ul>
     *          <li>Suggested start: .04</li>
     *          <li>Lower bound: 0</li>
     *          <li>Upper bound: Infinity</li>
     *        </ul>
     *
     * @param maxTime
     *        Maximum allowed execution time for this trajectory.
     *        Will be hard cut off after this time passes
     * 
     */
    public CMTrajectory(
            String pathName,
            CMAutonPoint[] controlPoints,
            CMRotation[] rotations,
            CMEvent[] events,
            double pointsPerMeter,
            double lookAheadMultiplier,
            TrajectoryPriority trajectoryPriority,
            double maxDesiredTranslationalVelocity,
            double desiredTranslationalAcceleration,
            double desiredTranslationalDecceleration,
            double minStartVelocity,
            boolean shouldStopAtEnd,
            double endVelocity,
            double distanceAtEndVelocity,
            double[] positionTolerance,
            double rotationSettleTime,
            double maxTime) {

        this.pathName = pathName;
        this.lookAheadMultiplier = lookAheadMultiplier;
        this.trajectoryPriority = trajectoryPriority;

        this.maxDesiredTranslationalVelocity = maxDesiredTranslationalVelocity;
        this.desiredTranslationalAcceleration = desiredTranslationalAcceleration;
        this.desiredTranslationalDecceleration = desiredTranslationalDecceleration;
        this.minStartVelocity = minStartVelocity;
        this.shouldStopAtEnd = shouldStopAtEnd;
        this.endVelocity = endVelocity;
        this.distanceAtEndVelocity = distanceAtEndVelocity;
        this.positionTolerance = positionTolerance;
        this.rotationSettleTime = rotationSettleTime;
        this.maxTime = maxTime;

        if (controlPoints.length == 0) {
            throw new ExceptionInInitializerError(
                    "For " + pathName + " CrowMotion paths need at least one control point");
        }
        if (this.endVelocity > maxDesiredTranslationalVelocity) {
            throw new ExceptionInInitializerError("For " + pathName
                    + " CrowMotion End Velocities must be less then or equal to max translational velocity");
        }
        if (this.maxTime <= 0) {
            throw new ExceptionInInitializerError("For " + pathName + " Max Time must be greater than 0");
        }
        if (this.lookAheadMultiplier <= 0) {
            throw new ExceptionInInitializerError("For " + pathName + " Look ahead multiplier must be greater than 0");
        }
        if (this.distanceAtEndVelocity < 0) {
            throw new ExceptionInInitializerError("For " + pathName + " Distance at end velocity must be at least 0");
        }
        if (this.minStartVelocity < 0) {
            throw new ExceptionInInitializerError("For " + pathName + " Min start velocity must be at least 0");
        }
        if (this.rotationSettleTime < 0) {
            throw new ExceptionInInitializerError("For " + pathName + " Time at final position must be at least 0");
        }
        if (this.maxDesiredTranslationalVelocity <= 0) {
            throw new ExceptionInInitializerError(
                    "For " + pathName + " Max desired translational velocity must be greater then 0");
        }
        if (this.desiredTranslationalAcceleration <= 0) {
            throw new ExceptionInInitializerError(
                    "For " + pathName + " Desired translational acceleration must be greater then 0");
        }
        if (this.desiredTranslationalDecceleration <= 0) {
            throw new ExceptionInInitializerError(
                    "For " + pathName + " Desired translational deceleration must be greater then 0");
        }
        this.drivebaseCircumference = CMConfig.getDrivebaseCircumference();
        this.maxModuleVelocity = CMConfig.getMaxPossibleAverageSwerveModuleMPS();

        this.futurePath = CMPathGenerator.generateCMPathAsync(pathName,
                controlPoints, rotations, events, pointsPerMeter);
        CMAutonPoint lastPoint = controlPoints[controlPoints.length - 1];

        this.endRobotState = new double[] { lastPoint.getX(), lastPoint.getY() };
    }
    
    /**
     * Creates a new {@code CMTranslationConfig} to define the translational motion
     * settings for a trajectory.
     *
     * <p>Each parameter directly affects how the robot follows and finishes the
     * path. These values should be tuned based on whether the trajectory is
     * stopping or non-stopping, the robot’s drivetrain limits, and the desired
     * balance between accuracy and speed.</p>
     ** @param pathName
     *        The name of the path. Used in logging.
     *
     * @param controlPoints
     *        Array of {@link CMAutonPoint} control points defining the Path.
     *        <ul>
     *          <li>If given 1 point, the path is a line from the robot to the point.</li>
     *          <li>If given 2 points, the path is a line from point 1 to point 2.</li>
     *          <li>If given 3 or more points, the path is a curve generated from the points.</li>
     *        </ul>
     *        <strong>Note:</strong> The Bezier curve algorithm only guarantees the 1st and last point
     *        are on the curve; others are control points. This is how all Bezier curve generation works,
     *        including PathPlanner, Choreo, and WPILib Trajectories.
     *
     * @param rotations
     *        Array of {@link CMRotation} objects specifying desired rotation
     *        deadlines (target angles, velocity profiles, and deadlines) along the trajectory.
     *
     * @param events
     *        Array of {@link CMEvent} triggers scheduled to occur at specific
     *        completion percentages of the trajectory.
     *
     * @param trajectoryPriority
     *        How to prioritize velocity when translational and rotational demands
     *        exceed the drivetrain’s capacity:
     *        <ul>
     *          <li>{@code PREFER_ROTATION}, Lowers translatinal speed however much is needed to allow for the rotation speed</li>
     *          <li>{@code PREFER_TRANSLATION}, Lowers rotation speed however much is needed to allow for the translational speed</li>
     *          <li>{@code SPLIT_PROPORTIONALLY}, Lowers both in proportion until the motion is possible</li>
     *        </ul>
     * @param maxDesiredTranslationalVelocity
     *        The maximum translational velocity the robot is allowed to reach while
     *        following the trajectory.
     *        <ul>
     *          <li>Suggested start value: 3.5 m/s</li>
     *          <li>Lower bound: &gt; 0</li>
     *          <li>Upper bound: robot’s maximum safe velocity</li>
     *        </ul>
     * @param desiredTranslationalAcceleration
     *        The maximum translational acceleration allowed while following the
     *        path. Controls how quickly the robot speeds up.
     *        <ul>
     *          <li>Suggested start: 3.5</li>
     *          <li>Lower bound: &gt; 0</li>
     *          <li>Upper bound: Robot’s maximum safe acceleration</li>
     *        </ul>
     *        <strong>Note:</strong> If odometry drift is occurring, decreasing
     *        acceleration will help prevent wheel slippage.
     *        
     * @param desiredTranslationalDecceleration
     *        The deceleration rate used only at the very end of the trajectory to
     *        smoothly reduce the robot’s velocity down to the specified end
     *        velocity by the time it reaches {@code distanceAtendVelocity}.
     *        <ul>
     *          <li>Suggested start: 3.5</li>
     *          <li>Upper bound: Robot’s maximum safe acceleration</li>
     *        </ul>
     *        <strong>Note:</strong> If odometry drift is occurring, decreasing
     *        deceleration will help prevent wheel slippage.
     *        <br>
     * @param minStartVelocity
     *        The initial velocity assigned to the robot when beginning the
     *        trajectory. Prevents excessively slow starts.
     *        <ul>
     *          <li>Suggested start: <b>2.0 m/s</b></li>
     *          <li>Lower bound: &gt; 0</li>
     *        </ul>
     *        This allows the robot to accelerate very quickly at the start, then
     *        transition to smoother acceleration to save time while limiting forces.
     *        <br>
     * @param shouldStopAtEnd
     *        Whether the robot’s velocity should be forced to 0 at the end of the
     *        trajectory.
     *        <ul>
     *          <li>true: Robot stops fully at the end.</li>
     *          <li>false: Robot continues moving, allowing a smooth transition
     *          to another trajectory or driver control.</li>
     *        </ul>
     *        <br>
     * @param endVelocity
     *        The velocity that the robot should be traveling at
     *        while it travels the distance specified by {@code distanceAtendVelocity}.
     *        <ul>
     *          <li>Stopping trajectories: Suggested ~= 0.08 m/s, or the maximum
     *          speed the robot can stop at “instantly."</li>
     *          <li>Non-stopping trajectories: Should be any value ≤
     *          {@code maxDesiredTranslationVelocityMeters}. Allows smooth pass-off to
     *          the next trajectory or driver control.</li>
     *        </ul>
     *        For stopping: decrease to increase accuracy, increase to save time.
     *        <br>
     * @param distanceAtEndVelocity
     *        The distance from the final target point at which the robot should
     *        already be traveling at {@code endVelocity}.
     *        <ul>
     *          <li>Stopping trajectories: Suggested ~= 0.05 m</li>
     *          <li>Non-stopping trajectories: Can be 0</li>
     *        </ul>
     *        Increase this distance to improve accuracy, decrease to save time.
     *        <br>
     * @param positionTolerance
     *        The positional tolerance that determines when the trajectory is
     *        considered complete.
     *        <ul>
     *          <li>Stopping trajectories: Suggested ~= ±0.01 m. Smaller values
     *          increase accuracy.</li>
     *          <li>Non-stopping trajectories: Suggested ≥ ±0.5 m. Looser
     *          tolerance allows smooth transitions to the next trajectory or driver
     *          control.</li>
     *        </ul>
     *        <br>
     
     * @param rotationSettleTime
     *        Time (s) the robot must remain within rotational tolerance before
     *        the trajectory is considered complete. Prevents overshoot from
     *        falsely ending the trajectory.
     *        <ul>
     *          <li>Suggested start: .04</li>
     *          <li>Lower bound: 0</li>
     *          <li>Upper bound: Infinity</li>
     *        </ul>
     *
     * @param maxTime
     *        Maximum allowed execution time for this trajectory.
     *        Will be hard cut off after this time passes
     * 
     */
    public CMTrajectory(
            String pathName,
            CMAutonPoint[] controlPoints,
            CMRotation[] rotations,
            CMEvent[] events,
            TrajectoryPriority trajectoryPriority,
            double maxDesiredTranslationalVelocity,
            double desiredTranslationalAcceleration,
            double desiredTranslationalDecceleration,
            double minStartVelocity,
            boolean shouldStopAtEnd,
            double endVelocity,
            double distanceAtEndVelocity,
            double[] positionTolerance,
            double rotationSettleTime,
            double maxTime) {
        
        this(pathName, controlPoints, rotations, events,
            CMConfig.getDefaultPointsPerMeter(),
            CMConfig.getDefaultLookAHeadMultiplier(),
            trajectoryPriority, maxDesiredTranslationalVelocity,
            desiredTranslationalAcceleration, desiredTranslationalDecceleration,
            minStartVelocity,
            shouldStopAtEnd,
            endVelocity, distanceAtEndVelocity, positionTolerance,
            rotationSettleTime, maxTime
        );
    }

    /**
     * Creates a new {@code CMTranslationConfig} to define the translational motion
     * settings for a trajectory.
     *
     * <p>Each parameter directly affects how the robot follows and finishes the
     * path. These values should be tuned based on whether the trajectory is
     * stopping or non-stopping, the robot’s drivetrain limits, and the desired
     * balance between accuracy and speed.</p>
     ** @param pathName
     *        The name of the path. Used in logging.
     *
     * @param controlPoints
     *        Array of {@link CMAutonPoint} control points defining the Path.
     *        <ul>
     *          <li>If given 1 point, the path is a line from the robot to the point.</li>
     *          <li>If given 2 points, the path is a line from point 1 to point 2.</li>
     *          <li>If given 3 or more points, the path is a curve generated from the points.</li>
     *        </ul>
     *        <strong>Note:</strong> The Bezier curve algorithm only guarantees the 1st and last point
     *        are on the curve; others are control points. This is how all Bezier curve generation works,
     *        including PathPlanner, Choreo, and WPILib Trajectories.
     *
     * @param rotations
     *        Array of {@link CMRotation} objects specifying desired rotation
     *        deadlines (target angles, velocity profiles, and deadlines) along the trajectory.
     *
     * @param events
     *        Array of {@link CMEvent} triggers scheduled to occur at specific
     *        completion percentages of the trajectory.
     *
     * @param trajectoryPriority
     *        How to prioritize velocity when translational and rotational demands
     *        exceed the drivetrain’s capacity:
     *        <ul>
     *          <li>{@code PREFER_ROTATION}, Lowers translatinal speed however much is needed to allow for the rotation speed</li>
     *          <li>{@code PREFER_TRANSLATION}, Lowers rotation speed however much is needed to allow for the translational speed</li>
     *          <li>{@code SPLIT_PROPORTIONALLY}, Lowers both in proportion until the motion is possible</li>
     *        </ul>
     * @param shouldStopAtEnd
     *        Whether the robot’s velocity should be forced to 0 at the end of the
     *        trajectory.
     *        <ul>
     *          <li>true: Robot stops fully at the end.</li>
     *          <li>false: Robot continues moving, allowing a smooth transition
     *          to another trajectory or driver control.</li>
     *        </ul>
     *        <br>
     * @param endVelocity
     *        The velocity that the robot should be traveling at
     *        while it travels the distance specified by {@code distanceAtendVelocity}.
     *        <ul>
     *          <li>Stopping trajectories: Suggested ~= 0.08 m/s, or the maximum
     *          speed the robot can stop at "instantly."</li>
     *          <li>Non-stopping trajectories: Should be any value ≤
     *          {@code maxDesiredTranslationVelocityMeters}. Allows smooth pass-off to
     *          the next trajectory or driver control.</li>
     *        </ul>
     *        For stopping: decrease to increase accuracy, increase to save time.
     *        <br>
     * @param distanceAtEndVelocity
     *        The distance from the final target point at which the robot should
     *        already be traveling at {@code endVelocity}.
     *        <ul>
     *          <li>Stopping trajectories: Suggested ~= 0.05 m</li>
     *          <li>Non-stopping trajectories: Can be 0</li>
     *        </ul>
     *        Increase this distance to improve accuracy, decrease to save time.
     *        <br>
     * @param positionTolerance
     *        The positional tolerance that determines when the trajectory is
     *        considered complete.
     *        <ul>
     *          <li>Stopping trajectories: Suggested ~= ±0.01 m. Smaller values
     *          increase accuracy.</li>
     *          <li>Non-stopping trajectories: Suggested ≥ ±0.5 m. Looser
     *          tolerance allows smooth transitions to the next trajectory or driver
     *          control.</li>
     *        </ul>
     *        <br>
     
     * @param rotationSettleTime
     *        Time (s) the robot must remain within rotational tolerance before
     *        the trajectory is considered complete. Prevents overshoot from
     *        falsely ending the trajectory.
     *        <ul>
     *          <li>Suggested start: .04</li>
     *          <li>Lower bound: 0</li>
     *          <li>Upper bound: Infinity</li>
     *        </ul>
     *
     * @param maxTime
     *        Maximum allowed execution time for this trajectory.
     *        Will be hard cut off after this time passes
     * 
     */
    public CMTrajectory(
            String pathName,
            CMAutonPoint[] controlPoints,
            CMRotation[] rotations,
            CMEvent[] events,
            TrajectoryPriority trajectoryPriority,
            boolean shouldStopAtEnd,
            double endVelocity,
            double distanceAtEndVelocity,
            double[] positionTolerance,
            double rotationSettleTime,
            double maxTime) {
        
        this(pathName, controlPoints, rotations, events,
            CMConfig.getDefaultPointsPerMeter(),
            CMConfig.getDefaultLookAHeadMultiplier(),
            trajectoryPriority, CMConfig.getDefaultMaxDesiredTranslationalVelocity(),
            CMConfig.getDefaultMaxDesiredTranslationalAcceleration(),
            CMConfig.getDefaultMaxDesiredTranslationalDecceleration(),
            CMConfig.getDefaultMinStartVelocity(),
            shouldStopAtEnd,
            endVelocity, distanceAtEndVelocity, positionTolerance,
            rotationSettleTime, maxTime
        );
    }
    
     /**
     * Creates a new {@code CMTranslationConfig} to define the translational motion
     * settings for a trajectory.
     *
     * <p>Each parameter directly affects how the robot follows and finishes the
     * path. These values should be tuned based on whether the trajectory is
     * stopping or non-stopping, the robot’s drivetrain limits, and the desired
     * balance between accuracy and speed.</p>
     ** @param pathName
     *        The name of the path. Used in logging.
     *
     * @param controlPoints
     *        Array of {@link CMAutonPoint} control points defining the Path.
     *        <ul>
     *          <li>If given 1 point, the path is a line from the robot to the point.</li>
     *          <li>If given 2 points, the path is a line from point 1 to point 2.</li>
     *          <li>If given 3 or more points, the path is a curve generated from the points.</li>
     *        </ul>
     *        <strong>Note:</strong> The Bezier curve algorithm only guarantees the 1st and last point
     *        are on the curve; others are control points. This is how all Bezier curve generation works,
     *        including PathPlanner, Choreo, and WPILib Trajectories.
     *
     * @param rotations
     *        Array of {@link CMRotation} objects specifying desired rotation
     *        deadlines (target angles, velocity profiles, and deadlines) along the trajectory.
     *
     * @param events
     *        Array of {@link CMEvent} triggers scheduled to occur at specific
     *        completion percentages of the trajectory.
     *
     * @param pointsPerMeter
     *        Resolution of the generated path, measured as number of interpolated
     *        path points per meter of travel. Higher values yield smoother paths but
     *        heavier computations.
     *        <ul>
     *          <li>Suggested start: 1.5 (Works well; if you want smoother, try 3)</li>
     *          <li>Lower bound: 1</li>
     *          <li>Upper bound: Infinity (anything above 3 is usually pointless)</li>
     *        </ul>
     * @param lookAheadMultiplier
     *        Multiplier used to determine which point along the path the robot should
     *        aim for.
     *        <ul>
     *          <li>Suggested value: 10</li>
     *          <li>Reasonable range: 7–10</li>
     *        </ul>
     *        Lower values improve path-following accuracy but increase side-to-side
     *        oscillation. Higher values reduce oscillation but cause the robot to
     *        cut corners more aggressively.
     * @param trajectoryPriority
     *        How to prioritize velocity when translational and rotational demands
     *        exceed the drivetrain’s capacity:
     *        <ul>
     *          <li>{@code PREFER_ROTATION}, Lowers translatinal speed however much is needed to allow for the rotation speed</li>
     *          <li>{@code PREFER_TRANSLATION}, Lowers rotation speed however much is needed to allow for the translational speed</li>
     *          <li>{@code SPLIT_PROPORTIONALLY}, Lowers both in proportion until the motion is possible</li>
     *        </ul>
     * @param maxDesiredTranslationalVelocity
     *        The maximum translational velocity the robot is allowed to reach while
     *        following the trajectory.
     *        <ul>
     *          <li>Suggested start value: 3.5 m/s</li>
     *          <li>Lower bound: &gt; 0</li>
     *          <li>Upper bound: robot’s maximum safe velocity</li>
     *        </ul>
     * @param desiredTranslationalAcceleration
     *        The maximum translational acceleration allowed while following the
     *        path. Controls how quickly the robot speeds up.
     *        <ul>
     *          <li>Suggested start: 3.5</li>
     *          <li>Lower bound: &gt; 0</li>
     *          <li>Upper bound: Robot’s maximum safe acceleration</li>
     *        </ul>
     *        <strong>Note:</strong> If odometry drift is occurring, decreasing
     *        acceleration will help prevent wheel slippage.
     *        
     * @param desiredTranslationalDecceleration
     *        The deceleration rate used only at the very end of the trajectory to
     *        smoothly reduce the robot’s velocity down to the specified end
     *        velocity by the time it reaches {@code distanceAtendVelocity}.
     *        <ul>
     *          <li>Suggested start: 3.5</li>
     *          <li>Upper bound: Robot’s maximum safe acceleration</li>
     *        </ul>
     *        <strong>Note:</strong> If odometry drift is occurring, decreasing
     *        deceleration will help prevent wheel slippage.
     *        <br>
     * @param minStartVelocity
     *        The initial velocity assigned to the robot when beginning the
     *        trajectory. Prevents excessively slow starts.
     *        <ul>
     *          <li>Suggested start: <b>2.0 m/s</b></li>
     *          <li>Lower bound: &gt; 0</li>
     *        </ul>
     *        This allows the robot to accelerate very quickly at the start, then
     *        transition to smoother acceleration to save time while limiting forces.
     *        <br>
     * @param shouldStopAtEnd
     *        Whether the robot’s velocity should be forced to 0 at the end of the
     *        trajectory.
     *        <ul>
     *          <li>true: Robot stops fully at the end.</li>
     *          <li>false: Robot continues moving, allowing a smooth transition
     *          to another trajectory or driver control.</li>
     *        </ul>
     *        <br>
     * @param positionTolerance
     *        The positional tolerance that determines when the trajectory is
     *        considered complete.
     *        <ul>
     *          <li>Stopping trajectories: Suggested ~= ±0.01 m. Smaller values
     *          increase accuracy.</li>
     *          <li>Non-stopping trajectories: Suggested ≥ ±0.5 m. Looser
     *          tolerance allows smooth transitions to the next trajectory or driver
     *          control.</li>
     *        </ul>
     *        <br>
     
     * @param rotationSettleTime
     *        Time (s) the robot must remain within rotational tolerance before
     *        the trajectory is considered complete. Prevents overshoot from
     *        falsely ending the trajectory.
     *        <ul>
     *          <li>Suggested start: .04</li>
     *          <li>Lower bound: 0</li>
     *          <li>Upper bound: Infinity</li>
     *        </ul>
     *
     * @param maxTime
     *        Maximum allowed execution time for this trajectory.
     *        Will be hard cut off after this time passes
     * 
     */
    public CMTrajectory(
            String pathName,
            CMAutonPoint[] controlPoints,
            CMRotation[] rotations,
            CMEvent[] events,
            TrajectoryPriority trajectoryPriority,
            double maxDesiredTranslationalVelocity,
            double desiredTranslationalAcceleration,
            double desiredTranslationalDecceleration,
            double minStartVelocity,
            boolean shouldStopAtEnd,
            double[] positionTolerance,
            double rotationSettleTime,
            double maxTime) {
        
        this(pathName, controlPoints, rotations, events,
            CMConfig.getDefaultPointsPerMeter(),
            CMConfig.getDefaultLookAHeadMultiplier(),
            trajectoryPriority, maxDesiredTranslationalVelocity,
            desiredTranslationalAcceleration, desiredTranslationalDecceleration,
            minStartVelocity,
            shouldStopAtEnd,
            CMConfig.getDefaultEndVelocity(),
            CMConfig.getDefaultDistenceAtEndVelocity(),
            positionTolerance,
            rotationSettleTime, maxTime
        );
    }

    /**
     * Creates a new {@code CMTranslationConfig} to define the translational motion
     * settings for a trajectory.
     *
     * <p>Each parameter directly affects how the robot follows and finishes the
     * path. These values should be tuned based on whether the trajectory is
     * stopping or non-stopping, the robot’s drivetrain limits, and the desired
     * balance between accuracy and speed.</p>
     * @param pathName
     *        The name of the path. Used in logging.
     *
     * @param controlPoints
     *        Array of {@link CMAutonPoint} control points defining the Path.
     *        <ul>
     *          <li>If given 1 point, the path is a line from the robot to the point.</li>
     *          <li>If given 2 points, the path is a line from point 1 to point 2.</li>
     *          <li>If given 3 or more points, the path is a curve generated from the points.</li>
     *        </ul>
     *        <strong>Note:</strong> The Bezier curve algorithm only guarantees the 1st and last point
     *        are on the curve; others are control points. This is how all Bezier curve generation works,
     *        including PathPlanner, Choreo, and WPILib Trajectories.
     *
     * @param rotations
     *        Array of {@link CMRotation} objects specifying desired rotation
     *        deadlines (target angles, velocity profiles, and deadlines) along the trajectory.
     *
     * @param events
     *        Array of {@link CMEvent} triggers scheduled to occur at specific
     *        completion percentages of the trajectory.
     *
     * @param trajectoryPriority
     *        How to prioritize velocity when translational and rotational demands
     *        exceed the drivetrain’s capacity:
     *        <ul>
     *          <li>{@code PREFER_ROTATION}, Lowers translatinal speed however much is needed to allow for the rotation speed</li>
     *          <li>{@code PREFER_TRANSLATION}, Lowers rotation speed however much is needed to allow for the translational speed</li>
     *          <li>{@code SPLIT_PROPORTIONALLY}, Lowers both in proportion until the motion is possible</li>
     *        </ul>
     * @param shouldStopAtEnd
     *        Whether the robot’s velocity should be forced to 0 at the end of the
     *        trajectory.
     *        <ul>
     *          <li>true: Robot stops fully at the end.</li>
     *          <li>false: Robot continues moving, allowing a smooth transition
     *          to another trajectory or driver control.</li>
     *        </ul>
     *        <br>
     * @param positionTolerance
     *        The positional tolerance that determines when the trajectory is
     *        considered complete.
     *        <ul>
     *          <li>Stopping trajectories: Suggested ~= ±0.01 m. Smaller values
     *          increase accuracy.</li>
     *          <li>Non-stopping trajectories: Suggested ≥ ±0.5 m. Looser
     *          tolerance allows smooth transitions to the next trajectory or driver
     *          control.</li>
     *        </ul>
     *        <br>
     
     * @param rotationSettleTime
     *        Time (s) the robot must remain within rotational tolerance before
     *        the trajectory is considered complete. Prevents overshoot from
     *        falsely ending the trajectory.
     *        <ul>
     *          <li>Suggested start: .04</li>
     *          <li>Lower bound: 0</li>
     *          <li>Upper bound: Infinity</li>
     *        </ul>
     *
     * @param maxTime
     *        Maximum allowed execution time for this trajectory.
     *        Will be hard cut off after this time passes
     * 
     */
    public CMTrajectory(
            String pathName,
            CMAutonPoint[] controlPoints,
            CMRotation[] rotations,
            CMEvent[] events,
            TrajectoryPriority trajectoryPriority,
            boolean shouldStopAtEnd,
            double[] positionTolerance,
            double rotationSettleTime,
            double maxTime) {
        
        this(pathName, controlPoints, rotations, events,
            CMConfig.getDefaultPointsPerMeter(),
            CMConfig.getDefaultLookAHeadMultiplier(),
            trajectoryPriority, CMConfig.getDefaultMaxDesiredTranslationalVelocity(),
            CMConfig.getDefaultMaxDesiredTranslationalAcceleration(),
            CMConfig.getDefaultMaxDesiredTranslationalDecceleration(),
            CMConfig.getDefaultMinStartVelocity(),
            shouldStopAtEnd,
            CMConfig.getDefaultEndVelocity(),
            CMConfig.getDefaultDistenceAtEndVelocity(),
            positionTolerance,
            rotationSettleTime, maxTime
        );
    }

    public void init() {
        endTime = -1;
        rotationSettleEndTime = -1;
        frameStartTime = -1;

        isComplete = false;
        preventTranslation = false;

        averageFrameTime = MetaConstants.loopPeriodSeconds;

        currentMinPointIndex = 1;
        goalPointIndex = 0;
        lastMinPointPathPoint = null;
        currentMinPointPathPoint = null;
        endPoint = null;
        lastMinPoint = null;
        currentMinPoint = null;
        nextMinPoint = null;

        lastDesiredVelocity = -1;
        velocityDelta = 0;
        initialDistanceToEnd = 0;
        shouldDecelerate = false;
        firstDecelerationFrame = true;

        lastRotationDeadlineIndex = -1;
        currentRotationDeadlineIndex = 0;
        rotationDeadline = null;

        desiredRotationDegrees = 0;
        maxRotationVelocityDegrees = 0;
        desiredRotationalAccelerationDegrees = 0;
        desiredRotatioanalDecelerationDegrees = 0;
        rotationDirection = 0;

        rotationCorrectionRangeDegrees = 0;
        maxRotationCorrectionVelocityDegrees = 0;
        minRotationVelocityToMove = 0;
        maxRotationToleranceDegrees = 0;
        decelerationBufferDegrees = 0;
        rotationDeadlineCompletePercent = 0;

        lastDesiredRotationalVelocity = 0;
        firstRotationDecelerationFrame = true;
        enteredCorrectionRange = false;
        shouldDecelerateRotation = false;
        initalRotationalVelocity = 0;
        initialDegreesToEnd = 0;

        lastTriggeredEventIndex = -1;

        if (events != null) {
            for (int i = 0; i < this.events.length; i++) {
                events[i].setHasBeenTriggered(false);
            }
        }
    }

    public void runTrajectoryFrame() {
        long currentTime = System.currentTimeMillis();

        if (frameStartTime != -1) {
            double frameTime = (currentTime - frameStartTime) / 1000.0;
            
            averageFrameTime = (averageFrameTime * .9) + (frameTime * .1);
        }
        frameStartTime = currentTime;

        double[] robotPosition = CMConfig.getRobotPositionMetersAndDegrees();
        double robotX = robotPosition[0];
        double robotY = robotPosition[1];
        double robotRot = robotPosition[2];

        this.isComplete = shouldEnd(robotX, robotY, robotRot, currentTime);
        if (!this.isComplete) {
            this.isComplete = shouldEnd(robotX, robotY, robotRot, currentTime);
        }
        Logger.recordOutput("CrowMotion/" + pathName + "/Status/IsComplete", this.isComplete);
        if (this.path != null) {
            if (this.isComplete) {
                if (events != null) {
                    for (int i = lastTriggeredEventIndex + 1; i < events.length; i++) {
                        CMEvent event = events[i];
                        if (!event.getHasBeenTriggered()) {
                            event.getEventFunction().run();
                            event.setHasBeenTriggered(true);
                            lastTriggeredEventIndex = i;
                        }
                    }
                }

                if (this.shouldStopAtEnd) {
                    CMConfig.setRobotVelocityMPSAndDPS(0, 0, 0);
                }
            } else {
                double[] currentVelocityComponents = CMConfig.getRobotVelocityMPSAndDPS();
                double currentVelocityMag = calculateMagnitude(currentVelocityComponents[0],
                        currentVelocityComponents[1]);
                if (this.lastDesiredVelocity == -1) {
                    this.lastDesiredVelocity = Math.max(currentVelocityMag, this.minStartVelocity);
                }

                if (lastMinPoint == null) {
                    this.lastMinPointPathPoint = path[0];
                    this.lastMinPoint = path[0].getTranslationalPoint();
                    this.currentMinPointPathPoint = path[1];
                    this.currentMinPoint = path[1].getTranslationalPoint();
                    this.nextMinPoint = path[2].getTranslationalPoint();
                    this.endPoint = path[path.length - 1];
                }

                if (rotationDeadline == null) {
                    if (rotationDeadlines == null || this.rotationDeadlines.length == 0) {
                        this.rotationDeadline = new CMRotation(robotRot,
                                0, .9, 5, false);
                    } else {
                        this.rotationDeadline = rotationDeadlines[currentRotationDeadlineIndex];
                    }
                    this.desiredRotationDegrees = rotationDeadline.getAngleDegrees();
                    this.maxRotationVelocityDegrees = this.rotationDeadline.getMaxRotationVelocityDegrees();
                    this.desiredRotationalAccelerationDegrees = this.rotationDeadline
                            .getDesiredRotationalAccelerationDegrees();
                    this.desiredRotatioanalDecelerationDegrees = this.rotationDeadline
                            .getDesiredRotationalDecelerationDegrees();
                    this.rotationDirection = this.rotationDeadline.getRotationDirection();
                    this.rotationCorrectionRangeDegrees = this.rotationDeadline.getAngleCorrectionRange();
                    this.maxRotationCorrectionVelocityDegrees = this.rotationDeadline
                            .getMaxRotationCorrectionVelocityDegrees();
                    this.minRotationVelocityToMove = this.rotationDeadline.getMinRotationVelocityToMoveDegrees();
                    this.maxRotationToleranceDegrees = this.rotationDeadline.getMaxToleranceDegrees();
                    this.decelerationBufferDegrees = this.rotationDeadline.getDecelerationBufferDegrees();
                    this.rotationDeadlineCompletePercent = this.rotationDeadline.getCompleteRotationPercent();
                }

                // Trejectory logic
                double desiredVelocityMag = calculateDesiredTranslationalVelocity(currentVelocityMag, robotX, robotY);
                double currentRotationalVelocity = Math.abs(currentVelocityComponents[2]);
                double rotationalVelocity = calculateDesiredRotationalVelocity(currentRotationalVelocity,
                        robotRot);

                double[] desaturatedVelocities = desaturateVelocities(desiredVelocityMag, Math.abs(rotationalVelocity));
                desiredVelocityMag = desaturatedVelocities[0];
                rotationalVelocity = desaturatedVelocities[1] * Math.signum(rotationalVelocity);

                double travelDistance = ((desiredVelocityMag + currentVelocityMag) / 2) * averageFrameTime;
                // Robot distence along path, distence to lastMinPoint + lastMinPoint distence
                // from start or
                double distanceToNext = calculateMagnitude(nextMinPoint.x - robotX, nextMinPoint.y - robotY);
                double distanceToLast = calculateMagnitude(lastMinPoint.x - robotX, lastMinPoint.y - robotY);

                while (distanceToNext - distanceToLast < 0 && this.currentMinPointIndex < path.length - 2) {
                    this.currentMinPointIndex++;
                    this.lastMinPointPathPoint = path[this.currentMinPointIndex - 1];
                    this.lastMinPoint = lastMinPointPathPoint.getTranslationalPoint();
                    this.currentMinPointPathPoint = path[this.currentMinPointIndex];
                    this.currentMinPoint = currentMinPointPathPoint.getTranslationalPoint();
                    this.nextMinPoint = path[this.currentMinPointIndex + 1].getTranslationalPoint();
                    distanceToNext = calculateMagnitude(nextMinPoint.x - robotX, nextMinPoint.y - robotY);
                    distanceToLast = calculateMagnitude(lastMinPoint.x - robotX, lastMinPoint.y - robotY);
                }

                double distanceFromStart = distanceToLast + this.lastMinPointPathPoint.getDistanceFromStart();
                double percentTravel = distanceFromStart / endPoint.getDistanceFromStart();

                if (rotationDeadlines != null && percentTravel >= this.rotationDeadlineCompletePercent) {
                    for (int i = currentRotationDeadlineIndex + 1; i < rotationDeadlines.length; i++) {
                        if (percentTravel >= this.rotationDeadlineCompletePercent) {
                            lastRotationDeadlineIndex = currentRotationDeadlineIndex;
                            currentRotationDeadlineIndex = i;
                            this.rotationDeadline = rotationDeadlines[currentRotationDeadlineIndex];
                            this.desiredRotationDegrees = rotationDeadline.getAngleDegrees();
                            this.maxRotationVelocityDegrees = this.rotationDeadline.getMaxRotationVelocityDegrees();
                            this.desiredRotationalAccelerationDegrees = this.rotationDeadline
                                    .getDesiredRotationalAccelerationDegrees();
                            this.desiredRotatioanalDecelerationDegrees = this.rotationDeadline
                                    .getDesiredRotationalDecelerationDegrees();
                            this.rotationDirection = this.rotationDeadline.getRotationDirection();
                            this.rotationCorrectionRangeDegrees = this.rotationDeadline.getAngleCorrectionRange();
                            this.maxRotationToleranceDegrees = this.rotationDeadline.getMaxToleranceDegrees();
                            this.decelerationBufferDegrees = this.rotationDeadline.getDecelerationBufferDegrees();
                            this.rotationDeadlineCompletePercent = this.rotationDeadline.getCompleteRotationPercent();
                        } else {
                            break;
                        }
                    }
                }

                if(events != null) {
                    for (int i = lastTriggeredEventIndex + 1; i < this.events.length - 1; i++) {
                        CMEvent event = events[i];
                        double triggerPercent = event.getEventTriggerPercent();
                        if (percentTravel >= triggerPercent && !event.getHasBeenTriggered()) {
                            event.getEventFunction().run();
                            event.setHasBeenTriggered(true);
                            lastTriggeredEventIndex = i;
                        } else {
                            break;
                        }
                    }
                }
                

                this.goalPointIndex = -1;
                Point2D.Double goalPointRangeEndPose = new Point2D.Double();
                double distanceToGoalPointEndRange = 0;
                for (int i = this.currentMinPointIndex; i < path.length - 2; i++) {
                    goalPointRangeEndPose = path[i].getTranslationalPoint();
                    distanceToGoalPointEndRange = calculateMagnitude(goalPointRangeEndPose.x - robotX,
                            goalPointRangeEndPose.y - robotY);
                    if (travelDistance * lookAheadMultiplier < distanceToGoalPointEndRange) {
                        goalPointIndex = i + 1;
                        break;
                    }
                }

                if (goalPointIndex == -1) {
                    goalPointIndex = path.length - 1;
                    goalPointRangeEndPose = endPoint.getTranslationalPoint();
                }

                double[] velocityDir = new double[] { goalPointRangeEndPose.x - robotX,
                        goalPointRangeEndPose.y - robotY };

                double[] velocityComponents = calculateComponentVelocities(velocityDir[0], velocityDir[1],
                        desiredVelocityMag);
                
                if (this.preventTranslation) {
                    CMConfig.setRobotVelocityMPSAndDPS(0, 0, rotationalVelocity);
                } else {
                    CMConfig.setRobotVelocityMPSAndDPS(velocityComponents[0], velocityComponents[1],
                            rotationalVelocity);
                }
                Logger.recordOutput("CrowMotion/" + pathName + "/TranslationVelocities/DesiredMag", desiredVelocityMag);
                Logger.recordOutput("CrowMotion/" + pathName + "/TranslationVelocities/ActualMag", currentVelocityMag);
                Logger.recordOutput("CrowMotion/" + pathName + "/RotationalVelocities/ActualMag", rotationalVelocity);
                Logger.recordOutput("CrowMotion/" + pathName + "/RotationalVelocities/ActualMag", currentRotationalVelocity);
                Logger.recordOutput("CrowMotion/" + pathName + "/Rotation/DesiredRotation", this.desiredRotationDegrees);
            }
            Logger.recordOutput("CrowMotion/" + pathName + "/Status/PathLoaded", true);
        } else {
            loadPath(currentTime);
        }
    }

    public boolean isCompleted() {
        return this.isComplete;
    }

    private boolean shouldEnd(double robotX, double robotY, double robotRot, long currentTime) {
        // If in x and y tolerance, end those motions but continue rotation tolerance
        // check for inputted time period
        boolean inTranslationalTolerance = Math.abs(robotX - this.endRobotState[0]) < this.positionTolerance[0] &&
                Math.abs(robotY - this.endRobotState[1]) < this.positionTolerance[1];
        boolean inRotTolorence = Math.abs(robotRot - desiredRotationDegrees) < this.maxRotationToleranceDegrees
                && this.rotationDeadlineCompletePercent == 1;

        if (inRotTolorence && this.rotationSettleEndTime == -1) {
            this.rotationSettleEndTime = currentTime + (long) (rotationSettleTime * 1000);
        } else if (!inRotTolorence && this.rotationSettleEndTime != -1) {
            this.rotationSettleEndTime = -1;
        }

        if (inTranslationalTolerance && this.shouldStopAtEnd) {
            this.preventTranslation = true;
        } else if (this.rotationSettleEndTime != -1 &&
                this.rotationSettleEndTime < currentTime) {
            this.preventTranslation = false;
        }

        boolean hasTimeElasped = endTime != -1 && currentTime >= endTime;

        boolean doneWithRot = rotationDeadlineCompletePercent != 1
                || (this.rotationSettleEndTime != -1 && this.rotationSettleEndTime < currentTime);
        return (inTranslationalTolerance && doneWithRot) || hasTimeElasped;
    }

    private double[] desaturateVelocities(double desiredTranslationMag, double desiredRotMag) {
        double velocityNeededForRotation = ((desiredRotMag / 360) * this.drivebaseCircumference);
        double requiredModuleVelocity = desiredTranslationMag + velocityNeededForRotation;

        if (requiredModuleVelocity > this.maxModuleVelocity) {
            switch (trajectoryPriority) {
                case SPLIT_PROPORTIONALLY: {
                    double proportion = maxModuleVelocity / (desiredTranslationMag + velocityNeededForRotation);
                    double newTranslationalVelocity = desiredTranslationMag * proportion;
                    double newRotationalLinearVelocity = maxModuleVelocity - newTranslationalVelocity;
                    double newRotationalVelocity = (newRotationalLinearVelocity * 360) / drivebaseCircumference;
                    return new double[] { newTranslationalVelocity, newRotationalVelocity };
                }
                case PREFER_TRANSLATION: {
                    if (desiredTranslationMag > maxModuleVelocity) {
                        return new double[] { desiredTranslationMag, 0 };
                    }
                    double newRotationalLinearVelocity = maxModuleVelocity - desiredTranslationMag;
                    double newRotationVelocity = (newRotationalLinearVelocity * 360) / drivebaseCircumference;
                    return new double[] { desiredTranslationMag, newRotationVelocity };
                }
                case PREFER_ROTATION: {
                    if (velocityNeededForRotation > maxModuleVelocity) {
                        return new double[] { 0, desiredRotMag };
                    }
                    double newTranslationalVelocity = maxModuleVelocity - velocityNeededForRotation;
                    return new double[] { newTranslationalVelocity, desiredRotMag };
                }
                default:
                    return new double[] { desiredTranslationMag, desiredRotMag };
            }
        }
        return new double[] { desiredTranslationMag, desiredRotMag };

    }

    private double[] calculateComponentVelocities(double deltaX, double deltaY,
            double translationalVelocityMagnitude) {
        double length = calculateMagnitude(deltaX, deltaY);
        if (length == 0)
            return new double[] { 0, 0 };
        double dx = (translationalVelocityMagnitude * deltaX) / length;
        double dy = (translationalVelocityMagnitude * deltaY) / length;
        return new double[] { dx, dy };
    }

    private double calculateDesiredTranslationalVelocity(double currentVelocityMag,
            double robotX, double robotY) {

        double distanceToEnd = 0.0;
        if (this.goalPointIndex >= path.length - 1) {
            Point2D.Double translationData = this.endPoint.getTranslationalPoint();
            distanceToEnd = calculateMagnitude(translationData.x - robotX, translationData.y - robotY);
        } else {
            double distanceFromRobotToMinPoint = calculateMagnitude(robotX - this.currentMinPoint.x,
                    robotY - this.currentMinPoint.y);
            distanceToEnd = (endPoint.getDistanceFromStart() - currentMinPointPathPoint.getDistanceFromStart()) +
                    distanceFromRobotToMinPoint;
        }

        if (!shouldDecelerate) {
            double distanceToStartDecelerating = (currentVelocityMag * currentVelocityMag
                    - this.endVelocity * this.endVelocity)
                    / (2 * this.desiredTranslationalDecceleration) + this.distanceAtEndVelocity;
            shouldDecelerate = distanceToEnd < distanceToStartDecelerating;
        }

        double desiredVelocity = 0;
        if (shouldDecelerate) {
            if (firstDecelerationFrame) {
                this.firstDecelerationFrame = false;
                this.velocityDelta = currentVelocityMag - this.endVelocity;
                this.initialDistanceToEnd = distanceToEnd;
            }
            double percent = ((distanceToEnd - this.distanceAtEndVelocity) / initialDistanceToEnd);
            desiredVelocity = Math.max((velocityDelta * clamp(0, 1, percent) + endVelocity), this.endVelocity);
        } else {
            firstDecelerationFrame = true;
            desiredVelocity = lastDesiredVelocity + this.desiredTranslationalAcceleration * this.averageFrameTime;
        }

        lastDesiredVelocity = Math.min(desiredVelocity, this.maxDesiredTranslationalVelocity);
        return lastDesiredVelocity;
    }

    private double calculateDesiredRotationalVelocity(double currentRotationalVelocity, double robotRot) {
        if (this.currentRotationDeadlineIndex != this.lastRotationDeadlineIndex) {
            this.lastDesiredRotationalVelocity = 0;
            this.firstRotationDecelerationFrame = true;
            this.enteredCorrectionRange = false;
            this.shouldDecelerateRotation = false;
            this.initalRotationalVelocity = 0;
            this.initialDegreesToEnd = 0;
        }
        double rawDegreesToGoal = robotRot - this.desiredRotationDegrees;
        rawDegreesToGoal = (rawDegreesToGoal + 540) % 360 - 180;
        double degreesToGoal = Math.abs(rawDegreesToGoal);
        double degreesToOffsetGoal = degreesToGoal - this.rotationCorrectionRangeDegrees;

        double dirToGoal = Math.signum(rawDegreesToGoal) * -1;
        if (!this.enteredCorrectionRange) {
            this.enteredCorrectionRange = degreesToGoal < this.rotationCorrectionRangeDegrees;
        }

        double realDesiredVelocity = 0;
        if (enteredCorrectionRange) {
            double desiredVelocity = Math.max(
                    maxRotationCorrectionVelocityDegrees * (degreesToGoal / this.rotationCorrectionRangeDegrees),
                    this.minRotationVelocityToMove);
            if (degreesToGoal < this.maxRotationToleranceDegrees) {
                desiredVelocity = 0;
            }
            realDesiredVelocity = desiredVelocity * dirToGoal;
        } else {
            double desiredVelocity = 0;
            if (!this.shouldDecelerateRotation) {
                double distanceToStartDecelerating = (currentRotationalVelocity * currentRotationalVelocity
                        - maxRotationCorrectionVelocityDegrees * maxRotationCorrectionVelocityDegrees)
                        / (2 * this.desiredRotatioanalDecelerationDegrees) + this.decelerationBufferDegrees;
                this.shouldDecelerateRotation = degreesToOffsetGoal < distanceToStartDecelerating;
            }

            if (this.shouldDecelerateRotation) {
                if (this.firstRotationDecelerationFrame) {
                    this.firstRotationDecelerationFrame = false;
                    this.initalRotationalVelocity = currentRotationalVelocity;
                    this.initialDegreesToEnd = degreesToOffsetGoal;
                }
                double percent = degreesToOffsetGoal / initialDegreesToEnd;
                desiredVelocity = Math.max(
                        (initalRotationalVelocity * clamp(0, 1, percent) + maxRotationCorrectionVelocityDegrees), 0);
            } else {
                this.firstDecelerationFrame = true;
                desiredVelocity = lastDesiredRotationalVelocity +
                        (this.desiredRotationalAccelerationDegrees * averageFrameTime);
            }
            double realRotationDirection = this.rotationDirection;
            if (this.rotationDirection == 0) {
                realRotationDirection = dirToGoal;
            }
            this.lastDesiredRotationalVelocity = desiredVelocity;
            realDesiredVelocity = desiredVelocity * realRotationDirection;
        }

        

        this.lastRotationDeadlineIndex = this.currentRotationDeadlineIndex;
        double rotationVelocityMag = Math.abs(realDesiredVelocity);
        double rotationDirSing = Math.signum(realDesiredVelocity);
        realDesiredVelocity = Math.min(rotationVelocityMag, maxRotationVelocityDegrees) * rotationDirSing;
        return realDesiredVelocity;
    }

    private double calculateMagnitude(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    @SuppressWarnings("unused")
    private double calculateMagnitudeRelative(double x, double y) {
        return (x * x + y * y);
    }

    private void loadPath(long currentTime) {
        if (pathGenResult == null && futurePath.isDone()) {
            pathGenResult = futurePath.getNow(new CMPathGenResult(null, null, null));
            path = pathGenResult.path;
            rotationDeadlines = pathGenResult.rotationDeadlines;
            events = pathGenResult.events;
            this.endTime = currentTime + (long) (this.maxTime * 1000);
            Logger.recordOutput("CrowMotion/" + pathName + "/Path", CMPathPoint.point2dToTranslation2D(path));
        }
    }

    private double clamp(double min, double max, double value) {
        return Math.min(Math.max(value, min), max);
    }

}
