package binarycrows.robot.Utils.Auton;

import java.util.function.Supplier;

import binarycrows.robot.StateRequest;
import binarycrows.robot.CrowMotion.Library.CMPathGenResult;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.StateRequestGroup.SequentialGroup;
import edu.wpi.first.math.geometry.Pose2d;

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
        this.builtAuton = stateRequestSupplier.get();
    }



}
