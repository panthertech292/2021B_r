// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Controller Mapping
    static final int kOperController = 1; // USB
    static final int kDriverController = 0; // USB

    public static final class DriveConstants {
        //Motor Mapping
        public static final int kFrontLeftMotor = 20;
        public static final int kBackLeftMotor = 1;
        public static final int kFrontRightMotor = 15;
        public static final int kBackRightMotor = 14;
        //Auto Move Constants
        public static final double kAutoForwardEncoderDistance = 144; //inches!
        public static final double kAutoBackwardEncoderDistance = 144; 
        public static final int kTestSwitch = 10;
        //Vision Constants
        public static final double kVisionAreaTarget = 1.4;
        //Sensor Constants
        public static final int kOpticalPort = 9; //DIO
        public static final int kSonarPort = 1;
        public static final double kSonarConversionFactor = 4.883; //Converts voltage to distance in inches. 
        //Drive Speeds
        public static final double kVisionAlignSpeed = .60; 
        public static final double kVisionForwardSpeed = .60; 
    }

    public static final class ShooterConstants {
        //Motor Mapping
        public static final int kShooterMotor = 13;
        public static final int kAimMotor = 7; 
        //Encoder Constants
        public static final int kShooterEncoderChannel1 = 0; //Placeholder value DIO
        public static final int kShooterEncoderChannel2 = 1; //Placeholder value DIO
        public static final int kAimEncoder1 = 2;
        public static final int kAimEncoder2 = 3;
        //Shooter Speed Constants
        public static final double kShooterStop = 0.0;
        public static final double kShooterHalf = 0.5;
        public static final double kShooterFull = 1.0;
        //DIO
        public static final int kAimLimitSwitch = 4;
        
    }
    public static final class BeltConstants{
        //Talon Mapping
        public static final int kFrontBeltMotor = 9; //Placeholder
        public static final int kBackBeltMotor = 2; //Placeholder
        public static final int kBottomBeltMotor = 77; //Placeholder

        public static final double kBeltForwardSpeed = -.65; //Placeholder
        public static final double kBeltBackwardSpeed = .2; //Placeholder

        public static final int kBallReadySensor = 12;//PlaceHolder
    }
}








