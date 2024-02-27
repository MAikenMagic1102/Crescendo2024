// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

/** double rotArmSetpoint
 *  double telescopeSetpoint
 */
public class ArmSetpoint {
    public double rotArmSetpoint;
    public double telescopeSetpoint;
    public ArmSetpoint(double rotArmSetpoint, double telescopeSetpoint){
        this.rotArmSetpoint = rotArmSetpoint;
        this.telescopeSetpoint = telescopeSetpoint;
    }
}