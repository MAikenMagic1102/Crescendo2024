// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm.ArmSetpoint;

/** Add your docs here. */
public final class Constants {
    public static final String canivoreBus = "1102";
    
    public static final class SwerveDrivetrain{
        public static final double maxSpeed = TunerConstants.kSpeedAt12VoltsMps;
    }
    
    public static final class Shooter{
        public static final int Shooter1_ID = 32;
        public static final int Shooter2_ID = 33;
        public static final int Feeder_ID = 31;
        public static final int noteSensor_DIO = 0;

        public static final boolean Feeder_Inverted = true;

        public static final boolean Shooter1_Inverted = false;
        public static final boolean Shooter2_Inverted = true;

        public static final double Shooter_kP = 0.1;
        public static final double Shooter_kI = 0.0;
        public static final double Shooter_kD = 0.0;

        public static final double kP = 0.1231;
        public static final double kS = 0.09347; // Amps
        public static final double kV = 0.1193;
        public static final double kA = 0.0408;

        public static final double Gear_Ratio = 84/74;

        public static InterpolatingTreeMap<Double,Double> shooterMap = new InterpolatingDoubleTreeMap();
        static {
                            //dist      RPM
            shooterMap.put(40.0, 65.0);
            shooterMap.put(158.0, 80.0);
            shooterMap.put(192.0, 85.0);
        }
    

    }

    public static final class Arm{
        public static final int leftArmMotorID = 25;
        public static final int rightArmMotorID = 26;
        public static final int telescopeMotorID = 27;

        public static final int armLatchPWM = 1;

        public static final boolean leftArm_Inverted = false;
        public static final boolean rightArm_Inverted = false;
        
        public static final double telescope_kP = 0.1;
        public static final double telescope_kI = 0.0;
        public static final double telescope_kD = 0.0;

        public static final double arm_kP = 0.1;
        public static final double arm_kI = 0.0;
        public static final double arm_kD = 0.0;

        //Testing
        public static final double kArmReduction = 200;
        public static final double kArmMass = 08.0; // Kilograms
        public static final double kArmLength = Units.inchesToMeters(30);
        public static final double kMinAngleRads = Units.degreesToRadians(-10);
        public static final double kMaxAngleRads = Units.degreesToRadians(90);

        public static final double ArmExtendSafe = 0.01;
        public static final ArmSetpoint INTAKE = new ArmSetpoint(-0.0455, 0.78);
        public static final ArmSetpoint STOW = new ArmSetpoint(-0.03, 0.76);
        public static final ArmSetpoint AMP = new ArmSetpoint(0.22, 1.79);
        public static final ArmSetpoint SUBWOOFER = new ArmSetpoint(-0.008, 0.750);
        public static final ArmSetpoint PODIUM = new ArmSetpoint(0.060, 1.95);
        public static final ArmSetpoint PRECLIMB = new ArmSetpoint(0.18, 1.8);
        public static final ArmSetpoint CLIMB = new ArmSetpoint(-0.02, 0.1);


        public static InterpolatingTreeMap<Double,Double> armMap = new InterpolatingDoubleTreeMap();
        static {
                        //Distance //Arm Position
            armMap.put(44.0, -0.008);
            armMap.put(82.0, 0.03);
            armMap.put(98.0, 0.045);
            armMap.put(114.0, 0.05);
            armMap.put(131.0, 0.062);            
            armMap.put(156.0, 0.0655);
            armMap.put(192.0, 0.069);
        }

        public static InterpolatingTreeMap<Double,Double> armMaxExtendMap = new InterpolatingDoubleTreeMap();
        static {
            armMaxExtendMap.put(0.0, 0.0);
        }
    }

    public static final class Limelight{
        public static final double limelight_kP = 0.0275;
        public static final double min_command = 0.05;
    }

    public static class FieldConstants {
        public static final Pose2d BLUE_SPEAKER = new Pose2d(Units.inchesToMeters(-1.5+12), Units.inchesToMeters(218.42), new Rotation2d(0));
        public static final Pose2d RED_SPEAKER = new Pose2d(Units.inchesToMeters(652.73-12), Units.inchesToMeters(218.42), new Rotation2d(Math.PI));
        public static final double BLUE_AUTO_PENALTY_LINE = 8.6; // X distance from origin to center of the robot almost fully crossing the midline
        public static final double RED_AUTO_PENALTY_LINE = 8; // X distance from origin to center of the robot almost fully crossing the midline

        public static final double NOTE_DIAMETER = 14; // Outer diameter of note

    }
}
