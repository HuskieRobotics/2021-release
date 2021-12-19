/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // TODO Make all constants final once testing w/ SmartDashboard
    public static final class TurretConstants{
        // TODO check CanIDs
        public static final int baseRotationMotorID = 5;
        public static final int hoodMotorID = 6;
        public static final int shootMotor1ID = 7;
        public static final int shootMotor2ID = 8;
        
        // TODO check center
        public static final double HOOD_CENTER = 0;

        // Maximum base position is 270 degrees in ticks
        public static double BASE_MAX = 4096 * (3/4);
        // 40 is just a random number tbh, no clue what this should be
        public static double HOOD_MAX = 40;

        // TODO tune PIDs
        public static double BASE_P = 2;
        public static double HOOD_P = 4;  
        
        // TODO check optimal shooter power
        
    }
}

