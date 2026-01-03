package binarycrows.robot.Utils.Auton;

import binarycrows.robot.StateRequest;
import binarycrows.robot.CrowMotion.Library.CMPathGenResult;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.StateRequestGroup.SequentialGroup;

public class Auton extends SequentialGroup {

    public AutonPoint startingPoint;


    public Auton(AutonPoint startingPoint, StateRequest... stateRequests) {
        super(StateRequestPriority.NORMAL, 30*1000, stateRequests);
        this.startingPoint = startingPoint;
    }



}
