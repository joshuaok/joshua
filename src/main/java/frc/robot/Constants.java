// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        private static final double In_To_M=.0254;
        private static final int Motor_Encoder_Codes_Per_Rev=2048;
        private static final double Diameter_Inches=5.0;
        private static final double Wheel_Diameter= Diameter_Inches * In_To_M;
        private static final double Wheel_Circumference= Wheel_Diameter * Math.PI;
        private static final double Gear_Ratio=12.75;
        public static final double Ticks_Per_Meter= ( Motor_Encoder_Codes_Per_Rev * Gear_Ratio)/(Wheel_Circumference);
        public static final double Meters_Per_Ticks= 1/Ticks_Per_Meter;
        
        // https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/identification-routine.html#running-the-identification-routine
        public static final double kTrackwidthMeters = 0.58;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double P = 0.15; // 0.15
        public static final double I = 0.001; //0.0005;
        public static final double D = 0; //0.1; 
        public static final double F = 0.05;
    }

public static final int Driver_controller_port=0;
public static final int Manipulator_controller_port=1;}

    
