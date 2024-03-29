// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Arm.ArmSetpoint;

/** Add your docs here. */
public final class Constants {
    public static final String canivoreBus = "1102";
    
    public static final class SwerveDrivetrain{

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
            shooterMap.put(0.0, 0.0);
        }
    

    }

    public static final class Arm{
        public static final int leftArmMotorID = 25;
        public static final int rightArmMotorID = 26;
        public static final int telescopeMotorID = 27;

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
        public static final ArmSetpoint INTAKE = new ArmSetpoint(-0.043, 0.84);
        public static final ArmSetpoint STOW = new ArmSetpoint(-0.03, 0.897);
        public static final ArmSetpoint AMP = new ArmSetpoint(0.195, 1.79);
        public static final ArmSetpoint SUBWOOFER = new ArmSetpoint(-0.008, 0.750);
        public static final ArmSetpoint PODIUM = new ArmSetpoint(0.062, 1.95);
        public static final ArmSetpoint PRECLIMB = new ArmSetpoint(0.18, 1.8);
        public static final ArmSetpoint CLIMB = new ArmSetpoint(0.00, 0.2);


        public static InterpolatingTreeMap<Double,Double> armMap = new InterpolatingDoubleTreeMap();
        static {
            armMap.put(44.0, -0.008);
            armMap.put(72.0, 0.03);
            armMap.put(106.0, 0.055);
            armMap.put(132.0, 0.065);
            armMap.put(145.0, 0.070);            
            armMap.put(166.0, 0.0744);
        }

        public static InterpolatingTreeMap<Double,Double> armMaxExtendMap = new InterpolatingDoubleTreeMap();
        static {
            armMaxExtendMap.put(0.0, 0.0);
        }
    }

    public static final class Limelight{
        public static final double limelight_kP = 0.015;
        public static final double min_command = 0.05;
    }
}
