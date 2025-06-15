package binarycrows.robot.SeasonCode.Subsystems.MotorTest;

import com.ctre.phoenix6.hardware.TalonFX;

public class MotorTestKraken implements MotorTestIO {
    TalonFX motorController;

    private double speed;

    public MotorTestKraken() {
        this.motorController = new TalonFX(0);
    }

    @Override
    public void setSpeed(double speed) {
        this.speed = speed;
        this.motorController.set(speed);
    }
}
