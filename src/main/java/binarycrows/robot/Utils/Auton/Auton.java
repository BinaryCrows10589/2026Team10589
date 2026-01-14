package binarycrows.robot.Utils.Auton;

import java.util.function.Supplier;

import binarycrows.robot.StateRequest;
import binarycrows.robot.CrowMotion.Library.CMPathGenResult;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.StateRequestGroup.SequentialGroup;

public class Auton extends SequentialGroup {

    public AutonPoint startingPoint;

    public Supplier<StateRequest[]> stateRequestSupplier;


    public Auton(AutonPoint startingPoint, Supplier<StateRequest[]> stateRequests) {
        super(StateRequestPriority.NORMAL, 30*1000, new StateRequest[]{});
        this.startingPoint = startingPoint;
        this.stateRequestSupplier = stateRequests;
    }

    /**
     * Actually gets the array of state requests to run (it's done like this to save on memory and startup time)
     * MUST be called before trying to run, otherwise nothing will happen!
     */
    public void buildAuton() {
        this.children = stateRequestSupplier.get();
    }



}
