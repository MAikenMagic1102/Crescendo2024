// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Arm.ArmSetpoint;

/** Add your docs here. */
public final class Constants {
    public static final String canivoreBus = "can2";
    
    public static final class SwerveDrivetrain{

    }
    
    public static final class Shooter{
        public static final int Shooter1_ID = 10;
        public static final int Shooter2_ID = 11;
        public static final int Feeder_ID = 13;
        public static final int noteSensor_DIO = 0;

        public static final boolean Feeder_Inverted = true;

        public static final boolean Shooter1_Inverted = true;
        public static final boolean Shooter2_Inverted = false;

        public static final double Shooter_kP = 0.1;
        public static final double Shooter_kI = 0.0;
        public static final double Shooter_kD = 0.0;

        public static final double Gear_Ratio = 84/74;
    }

    public static final class Arm{
        public static final int leftArmMotorID = 31;
        public static final int rightArmMotorID = 32;
        public static final int telescopeMotorID = 33;

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

        public static final double ArmExtendSafe = 0.025;
        public static final ArmSetpoint INTAKE = new ArmSetpoint(-0.02, 0.06);
        public static final ArmSetpoint STOW = new ArmSetpoint(0.025, 0.005);
        public static final ArmSetpoint AMP = new ArmSetpoint(0.25, 0.06);
        public static final ArmSetpoint SUBWOOFER = new ArmSetpoint(0.0, 0.0);
    }

    public static final class Limelight{

    }
}
