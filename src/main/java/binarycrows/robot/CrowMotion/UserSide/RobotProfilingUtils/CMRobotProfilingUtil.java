package binarycrows.robot.CrowMotion.UserSide.RobotProfilingUtils;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import binarycrows.robot.CrowMotion.Library.StatisticUtil;
import binarycrows.robot.CrowMotion.UserSide.CMConfig;

public class CMRobotProfilingUtil {
    
    
    public class ProfileMaxPossibleTranslationalVelocityMPSMaxPossibleAverageSwerveModuleMPS {
        /*
        * 0. Before running this profiler, ensure that your wheel circumference is configured 
            accuretly otherwise the profile will not be accurette. Furthermore, your CrowMotionConfig
            must have been configured(except for your RobotProfile of cource).
            In addition you should have a fully charged, good batter in the robot
        * 1. Call this method in the TeleopPeriod method of of the Robot.java file
        * 2. Ensure that all defualt commands partaining to the swerve subsystem
        *  are disabled temporarly otherwise this will not work as the defualt command 
        *  will hold control of the swerve subsystem, preventing this from working.
        * 3. Place the robot on your practice field carpet. Ensure that the robot has 
        *    plenty of room to fully accelerate and hold its max speed. 
        *    The robot will drive forward untill it holds its max speed for 13 frames(50 frames per second)
        *    second starting on robot enable
        * WARNING, The robot wil accelerate fast, be carfull to ensure it will not tip over. 
        * If needed this method lets you limit your acceleration but that will result in more room being needed. 
        * 5. This function will log the max translational velocity to Smartdashboard.
            You will need to input this value to your RobotProfile intence that you provide to your CrowMotionConfig   
        * 6. The robot will start to decelerate slowly after the profile, just disable the robot whenver you are confinate it will not tip over
        */
        private static double desiredVelocity; 
        private static double lastStartTime = 0;
        private static double frameTime;
        private static double framesAtCurrentVelocity;
        private static double lastVelocity = 0;
        private static boolean finished = false;
        private static double startTime = 0;
        public static void profileMaxPossibleTranslationalVelocityMPSAndMaxPossibleAverageSwerveModuleMPS(double limitedAcceleration) {
            if(startTime == 0) {
                startTime = System.currentTimeMillis();
            }
            if(lastStartTime == 0) {
                lastStartTime = System.currentTimeMillis();
            }
            double currentVelocity = Math.sqrt(Math.pow(CMConfig.getRobotVelocityMPSAndDPS()[0], 2) + 
                Math.pow(CMConfig.getRobotVelocityMPSAndDPS()[1], 2));
            frameTime = (System.currentTimeMillis() - lastStartTime) / 1000.0;
            lastStartTime = System.currentTimeMillis();
            if(limitedAcceleration == -1 && !finished) {
                desiredVelocity = 10;
            } else if (!finished) {
                desiredVelocity += limitedAcceleration * frameTime;
                if(desiredVelocity > 10) {
                    desiredVelocity = 10;
                }
            } else {
                if(desiredVelocity > 0) {
                    desiredVelocity -= (currentVelocity * frameTime);
                    if(Math.abs(currentVelocity - 0) < .2) {
                        desiredVelocity = 0; 
                    }
                }
            }
            CMConfig.setRobotVelocityMPSAndDPS(desiredVelocity, 0, 0);
            
            boolean inTolorence = Math.abs(currentVelocity - lastVelocity) < .005;
            lastVelocity = currentVelocity;
            
            if(inTolorence && currentVelocity != 0) {
                framesAtCurrentVelocity++;
            } else {
                framesAtCurrentVelocity = 0;
            }

            if(framesAtCurrentVelocity >= 13 && !finished) {
                SmartDashboard.putNumber("CrowMotion/RobotProfile/MaxPossibleTranslationalVelocityMPS", currentVelocity);
                SmartDashboard.putNumber("CrowMotion/RobotProfile/MaxPossibleAverageSwerveModuleMPS", Math.abs(CMConfig.getAverageSwerveModuleVelocityMPS()));
                finished = true;
            }
        }

        public static void profileMaxPossibleTranslationalVelocityMPSAndMaxPossibleAverageSwerveModuleMPS() {
            profileMaxPossibleTranslationalVelocityMPSAndMaxPossibleAverageSwerveModuleMPS(-1);
        }
    }

    public class ProfileMaxPossibleRotationalVelocityDPS {
        /*
        * 0. Before running this profiler, ensure that your wheel circumference is configured 
            accuretly otherwise the profile will not be accurette. Furthermore, your CrowMotionConfig
            must have been configured(except for your RobotProfile of cource).
            In addition you should have a fully charged, good batter in the robot
        * 1. Call this method in the TeleopPeriod method of of the Robot.java file
        * 2. Ensure that all defualt commands partaining to the swerve subsystem
        *  are disabled temporarly otherwise this will not work as the defualt command 
        *  will hold control of the swerve subsystem, preventing this from working.
        * 3. Place the robot on your practice field carpet. Ensure that the robot has 
        *    plenty of room to fully accelerate and hold its max speed. While rotating. 
             Remember that the robot wil drift some in translation while rotating
        *    The robot will drive forward untill it holds its max speed for 13 frames(50 frames per second)
        *    second starting on robot enable
        * WARNING, The robot wil accelerate fast, be carfull to ensure it will not tip over. 
        * If needed this method lets you limit your acceleration but that will result in more room being needed. 
        * 5. This function will log the max rotational velocity to Smartdashboard.
            You will need to input this value to your RobotProfile intence that you provide to your CrowMotionConfig   
        * 6. The robot will start to decelerate slowly after the profile, just disable the robot whenver you are confinate it will not tip over
        */
        private static double desiredVelocity = 0; 
        private static double lastStartTime = 0;
        private static double frameTime;
        private static double framesAtCurrentVelocity = 0;
        private static double lastVelocity = 0;
        private static boolean finished = false;
        private static double startTime = 0;
        public static void profileMaxPossibleRotationalVelocityDPS(double limitedAcceleration) {
            if(startTime == 0) {
                startTime = System.currentTimeMillis();
            }
            if(lastStartTime == 0) {
                lastStartTime = System.currentTimeMillis();
            }
            double currentVelocity = CMConfig.getRobotVelocityMPSAndDPS()[2];
            frameTime = (System.currentTimeMillis() - lastStartTime) / 1000.0;
            lastStartTime = System.currentTimeMillis();
            if(limitedAcceleration == -1 && !finished) {
                desiredVelocity = 1000;
            } else if (!finished) {
                desiredVelocity += limitedAcceleration * frameTime;
                if(desiredVelocity > 10000) {
                    desiredVelocity = 10000;
                }
            } else {
                if(desiredVelocity > 0) {
                    desiredVelocity -= (currentVelocity * frameTime);
                    if(Math.abs(currentVelocity - 0) < 20) {
                        desiredVelocity = 0; 
                    }
                }
            }
            CMConfig.setRobotVelocityMPSAndDPS(0, 0, desiredVelocity);
            boolean inTolorence = Math.abs(currentVelocity - lastVelocity) < 1;
            lastVelocity = currentVelocity;
            
            if(inTolorence && currentVelocity != 0) {
                framesAtCurrentVelocity++;
            } else {
                framesAtCurrentVelocity = 0;
            }

            if(framesAtCurrentVelocity >= 13 && !finished) {
                SmartDashboard.putNumber("CrowMotion/RobotProfile/MaxPossibleRotationalVelocityDPS", currentVelocity);
                finished = true;
            }
        }

        public static void profileMaxPossibleRotationalVelocityDPS() {
            profileMaxPossibleRotationalVelocityDPS(-1);
        }
    }
    /* 
    public class ProfileAcceleration {
        private static double intervalStartTime = 0;
        private static boolean intervalFinished = false;
        private static double desiredVelocity = 0;
        private static double intervalStartVelocity = 0;
        private static ArrayList<double[]> velocityVSAccelerationTabe = new ArrayList<double[]>();
        private static double lastStartTime = 0;
        private static double frameTime;
        private static boolean slowDown = false;

        public static void profileAcceleration(double accelerationVelocityInterval) {
            if(lastStartTime == 0) {
                lastStartTime = System.currentTimeMillis();
            }
            frameTime = (System.currentTimeMillis() - lastStartTime) / 1000;
            lastStartTime = System.currentTimeMillis(); 
            double currentVelocity = Math.sqrt(Math.pow(CrowMotionConfig.getRobotVelocityMPSandDPS()[0], 2) + 
                Math.pow(CrowMotionConfig.getRobotVelocityMPSandDPS()[1], 2));
            intervalFinished = Math.abs(currentVelocity - desiredVelocity) < .05 && !slowDown;
            Logger.recordOutput("CrowMotion/RobotProfile/AccelerationFormula(VelocityVSAcceleration)/IntervalFinished", intervalFinished);
            if(intervalFinished) {
                if(desiredVelocity != 0) {
                    velocityVSAccelerationTabe.add(new double[] {
                        (intervalStartVelocity+currentVelocity) / 2,
                        ((currentVelocity - intervalStartVelocity) /
                        ((System.currentTimeMillis()-intervalStartTime) / 1000)) 
                    });
                }
                intervalStartTime = System.currentTimeMillis();
                intervalStartVelocity = currentVelocity;
                intervalFinished = false;
                desiredVelocity = currentVelocity + accelerationVelocityInterval;
                if(desiredVelocity >= CrowMotionConfig.getRobotProfile().getMaxPossibleTranslationalVelocityMPS()) {
                    desiredVelocity = CrowMotionConfig.getRobotProfile().getMaxPossibleTranslationalVelocityMPS();
                }
                
                if(currentVelocity >= CrowMotionConfig.getRobotProfile().getMaxPossibleTranslationalVelocityMPS()-.3 && !slowDown) {
                    SmartDashboard.putString("CrowMotion/RobotProfile/AccelerationFormula(VelocityVSAcceleration)/Formula y=",
                    StatisticUtil.calculateRegressionLineFormula(velocityVSAccelerationTabe));
                    slowDown = true;
                }
            }
            if(!intervalFinished && !slowDown) {
                CrowMotionConfig.setRobotVelocityMPSandDPS(desiredVelocity, 0, 0);
            } else if(slowDown) {
                desiredVelocity -= (currentVelocity * frameTime/2);
                if(Math.abs(currentVelocity - 0) < .2) {
                    desiredVelocity = 0; 
                }
                CrowMotionConfig.setRobotVelocityMPSandDPS(desiredVelocity, 0, 0);
            }
        }
    }
    */
}
