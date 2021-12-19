/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class RobotConstants {
        public static final int COMPRESSOR_CANID = 19;
        public static String state = "Idle";
    }

    public static final class DriveConstants {
        public static final int[] DRIVETRAIN_LEFT_TALONS = {13, 14, 15};
        public static final int[] DRIVETRAIN_RIGHT_TALONS = {0, 1, 2};
        public static final int JOYSTICK_0 = 0;
        public static final int JOYSTICK_1 = 1;
        public static final int NUM_JOYSTICK_BUTTONS = 13;
        public static final int JOYSTICK_INVERT_BUTTON = 3;
        public static final int JOYSTICK_HIGHGEAR_BUTTON = 0;

        public static final double DEADZONE = 0.1;
        public static final double LOWGEAR_TRANSLATION_POWER = 1;
        public static final double LOWGEAR_TURNING_POWER = 1;
        public static final double HIGHGEAR_TRANSLATION_POWER = 2;
        public static final double HIGHGEAR_TURNING_POWER = 2;

        public static final int LEFT_ENCODER_CANID = 17;
        public static final int RIGHT_ENCODER_CANID = 18;
        //check CANID for PIEGON SENSOR
        public static final int PIEGON_CANID = 16; 

        public static final int GEAR_SHIFT_CHANNEL = 0;

        public static final double ENCODER_DISTANCE_PER_COUNT_FEET = (0.5 * Math.PI / 360);

        public static final double TRAPEZOIDAL = 0.03;
        public static final double RATE_LIMIT = 0.5;

        public static final double TICKS_TO_DEGREES = (360 / 4096);
        public static final double DEGREES_TO_TICKS = (4096 / 360);
    }

    public static final class AutoConstants {
        public static final double P_DRIVE = 0.65;
        public static double P_TURN = 0.0017;
        public static double P_LIMELIGHTTURN = 0.0155;

        // TODO edit this number
        public static final double DRIVE_STRAIGHT_DISTANCE = 6.5;
        public static final double DRIVE_45TURN_DISTANCE = 45; 
        public static final double DRIVE_90TURN_DISTANCE = 90;
        public static final double DRIVE_NEG90TURN_DISTANCE = -90;  
        public static final double DRIVE_180TURN_DISTANCE = 180; 
        public static final double DRIVE_270TURN_DISTANCE = 270;
        public static final double DRIVE_360TURN_DISTANCE = 360; 

        public static final double DRIVE_TOLERANCE = 0.1;
        public static final double TURN_TOLERANCE = 3;
        public static final double LIME_TOLERANCE = 0.75;

    }
    public static final class ShooterConstants {
        public static final int shooterMotor1CanID = 10;
        public static final int shooterMotor2CanID = 11;

        public static double shootingSpeed = .98; //0.7 green 
        public static final double shuttlingSpeed = .5;
        public static final double manualShooterSpeed = 0.5;
    }



    public static final class StorageConstants {
        public static final int storageMotor1CanID = 5;
        public static final int storageMotor2CanID = 6;
        public static final int bottomSensorChannel = 0;
        public static final int topSensorChannel = 1;

        public static final double storageFastSpeed = .4;
        public static final double storageSlowSpeed = .2;
        public static final double storageDelay = .2;
        public static final int cycleLength = 20691;

        public static final double manualStorageSpeed = .5; 
        
    }

    public static final class CollectorConstants {
        public static final int collectorMotorCanID = 4;
        public static final int collectorSolenoidChannel = 1;

        public static final double intakeSpeed = .5;
        public static final double autoUnjamTime = 2;
        public static final double outtakeSpeed = -1;
        public static final double manualIntakeSpeed = .5;
        public static final int minIntakeEncoderVelocity = 1000;
    }

    public static final class RobotGlobal{
        public static String state = "Idle";
    } 
}