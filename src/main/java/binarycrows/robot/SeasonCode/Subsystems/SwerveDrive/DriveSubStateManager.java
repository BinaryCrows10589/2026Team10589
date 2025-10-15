package binarycrows.robot.SeasonCode.Subsystems.SwerveDrive;

import java.util.ArrayList;
import java.util.HashMap;

import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestStatus;
import binarycrows.robot.SeasonCode.Constants.ElevatorConstants;
import binarycrows.robot.SeasonCode.Constants.SwerveDriveConstants;
import binarycrows.robot.SeasonCode.Subsystems.Elevator.ElevatorDualKrakenIO;
import binarycrows.robot.Utils.LogIOInputs;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

public class DriveSubStateManager extends SubStateManager<DriveStateRequest> {
    
    SwerveModule swerveModule;

    private double currentVoltageTableTargetValue = 0;
    private double voltageTableStep = 0.1;
    private double voltageTableRecordingTime = 2;
    private double voltageTableMaxValue = SwerveDriveConstants.maxDriveMotorVoltage;
    private double voltageTableMinValue = 0;
    private boolean startedRecordingDeceleration = false;
    private ArrayList<Double> accelerationTable = new ArrayList<Double>();
    private ArrayList<Double> decelerationTable = new ArrayList<Double>();

    private Timer voltageRecordingTimer = new Timer();

    public DriveSubStateManager() {
        super();
        swerveModule = new SwerveModule(new SwerveModuleTalonFXIO("FrontLeft"), "FrontLeft");
    }

    private void recordVoltageTableValue() {
        ArrayList<Double> targetArray = (startedRecordingDeceleration ? decelerationTable : accelerationTable);
        targetArray.add(currentVoltageTableTargetValue);
        targetArray.add(swerveModule.getModuleState().speedMetersPerSecond);
    }

    @Override
    public void periodic() {

        // Resolve pending state request
        if (this.activeStateRequest.getStatus() == StateRequestStatus.PENDING) {
            switch (this.activeStateRequest.getStateRequestType()) {
                case CONSTRUCT_VOLTAGE_TABLE:
                    voltageRecordingTimer.reset();
                    break;
                default:
                    swerveModule.stopModuleDrive();
                    break;
            }
            this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);

        }
        switch (this.activeStateRequest.getStateRequestType()) {
            case CONSTRUCT_VOLTAGE_TABLE:

                LogIOInputs.logToStateTable(currentVoltageTableTargetValue, "DriveSubsystem/VoltageTableTargetValue");
                LogIOInputs.logToStateTable(voltageRecordingTimer.get(), "DriveSubsystem/VoltageRecordingTimeElapsed");

                if (voltageRecordingTimer.hasElapsed(voltageTableRecordingTime)) { // It's time to record, buster brown!

                    if (!startedRecordingDeceleration) { // Still accelerating

                        if (currentVoltageTableTargetValue >= voltageTableMaxValue) { // Start bringing the speed back down since we've hit the max
                            startedRecordingDeceleration = true;
                            recordVoltageTableValue();
                        }

                        else { // Record the next value we need
                            recordVoltageTableValue();
                            currentVoltageTableTargetValue += voltageTableStep;
                            swerveModule.setDesiredModuleDriveVoltage(currentVoltageTableTargetValue);
                            voltageRecordingTimer.reset();
                        }
                    }
                    else { // Now decelerating
                        if (currentVoltageTableTargetValue <= voltageTableMinValue) { // Stop since we've hit the minimum
                            startedRecordingDeceleration = false;
                            this.returnToDefaultState();
                            recordVoltageTableValue();
                            LogIOInputs.logToStateTable(accelerationTable, "DriveSubsystem/AccelerationTable");
                            LogIOInputs.logToStateTable(decelerationTable, "DriveSubsystem/DecelerationTable");
                        }
                        else { // Record the next value we need
                            recordVoltageTableValue();
                            currentVoltageTableTargetValue -= voltageTableStep;
                            swerveModule.setDesiredModuleDriveVoltage(currentVoltageTableTargetValue);
                            voltageRecordingTimer.reset();
                        }
                    }
                }

                
                break;
            default:
                break;
        }

    }
}
