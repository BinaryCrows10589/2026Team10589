package binarycrows.robot.SeasonCode.Utils;

public class Shooting { //TODO: WIP
    public static boolean isShooting = false;
    public static boolean isForceShooting = false;
    public static boolean canShoot = false;

    public static boolean getCanShoot() {
        return false;
    }

    public static void periodic() {
        
    }

    public static boolean getShooting() {
        return isShooting && canShoot || isForceShooting;
    }
}
