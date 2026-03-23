package binarycrows.robot.Utils.Auton;

import java.util.function.Supplier;

import binarycrows.robot.StateRequest;
import binarycrows.robot.CrowMotion.Library.CMPathGenResult;
import binarycrows.robot.CrowMotion.UserSide.CMStateRequest;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.StateRequestGroup.SequentialGroup;
import edu.wpi.first.math.geometry.Pose2d;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveStateRequest;

public class Auton {

    public Pose2d startingPoint;

    public Supplier<SequentialGroup> stateRequestSupplier;

    public SequentialGroup builtAuton;


    public Auton(Pose2d startingPoint, Supplier<SequentialGroup> stateRequests) {
        this.startingPoint = startingPoint;
        this.stateRequestSupplier = stateRequests;
    }

    /**
     * Actually gets the array of state requests to run (it's done like this to save on memory and startup time)
     * MUST be called before trying to run, otherwise nothing will happen!
     */
    public void buildAuton() {
        if (this.builtAuton != null) System.out.println("Warning: rebuilding auton that was already built!");
        this.builtAuton = stateRequestSupplier.get();
        for (StateRequest request : builtAuton.getStateRequests()) {
            if (request.getStateRequestType() == DriveStateRequest.DRIVE_CROWMOTION) {
                System.out.println(((CMStateRequest)request).getTrajectory());
                ((CMStateRequest)request).getTrajectory().init();
            }
        }
    }

    public void rebuildAuton() {
        this.builtAuton = stateRequestSupplier.get();
        for (StateRequest request : builtAuton.getStateRequests()) {
            if (request.getStateRequestType() == DriveStateRequest.DRIVE_CROWMOTION) {
                System.out.println(((CMStateRequest)request).getTrajectory());
                ((CMStateRequest)request).getTrajectory().init();
            }
        }
    }



}
