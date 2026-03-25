package binarycrows.robot.SeasonCode.SubStateManagers.Intake.Rollers;

import java.util.function.Supplier;

import binarycrows.robot.StateRequest;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.Enums.StateRequestStatus;
import binarycrows.robot.SeasonCode.Constants.IntakeConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Rollers.IntakeRollersIO.IntakeRollersOutputs;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveSubStateManager;
import binarycrows.robot.Utils.LoggingUtils;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class IntakeRollersSubStateManager extends SubStateManager<IntakeRollersStateRequest> {
    
    private IntakeRollersIO intakeRollersIO;
    private IntakeRollersOutputs outputs;

    private Supplier<ChassisSpeeds> chassisSpeeds;

    public IntakeRollersSubStateManager() {
        super(new StateRequest<IntakeRollersStateRequest>(IntakeRollersStateRequest.OFF, StateRequestPriority.NORMAL));

        outputs = new IntakeRollersOutputs();
        intakeRollersIO = new IntakeRollersTalonFXS(outputs);

    }

    @Override
    public void setupSuppliers() {
        chassisSpeeds = DriveSubStateManager.getInstance()::getChassisSpeeds;

    }

    @Override
    public void recieveStateRequest(StateRequest<IntakeRollersStateRequest> request) {
        if (request.getStateRequestType() == IntakeRollersStateRequest.INVERT) {
            request = new StateRequest<IntakeRollersStateRequest>(
                (activeStateRequest.getStateRequestType() == IntakeRollersStateRequest.OFF) ? IntakeRollersStateRequest.INTAKING : IntakeRollersStateRequest.OFF, // If we're already off, start intaking. Otherwise, shut off.
                request.getPriority());
        }
        super.recieveStateRequest(request);
    }

    @Override
    public void periodic() {
        LoggingUtils.logObject("Intake/Rollers/Outputs", outputs);
        intakeRollersIO.update();
        switch (activeStateRequest.getStateRequestType()) {
            case OVERDRIVE:
                intakeRollersIO.setRotorVoltage(getDrivenIntakeVoltage() + IntakeConstants.Rollers.overdriveVoltage);
                this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                break;
            case INTAKING:
                intakeRollersIO.setRotorVoltage(getDrivenIntakeVoltage());
                this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                break;
            case OFF:
                intakeRollersIO.setRotorVoltage(0);
                this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                break;
            case REVERSE:
                intakeRollersIO.setRotorVoltage(-IntakeConstants.Rollers.intakingMotorVoltage);
                this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                break;
            default:
                break;
        }
    }

    private double getDrivenIntakeVoltage() {
        ChassisSpeeds speeds = chassisSpeeds.get();

        return Math.min(
                        Math.max(
                            IntakeConstants.Rollers.intakingMotorVoltage 
                                * Math.sqrt(speeds.vxMetersPerSecond * speeds.vxMetersPerSecond),
                            IntakeConstants.Rollers.intakeWheelMinVoltage),
                    IntakeConstants.Rollers.intakeWheelMaxVoltage);
    }

    public String toString() {
        return "IntakeRollersSubStateManager";
    }
}
