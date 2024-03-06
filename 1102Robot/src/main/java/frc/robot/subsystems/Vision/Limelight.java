// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class Limelight {
NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-magic");

//Degrees (vertical)
double limelightMountAngleDegrees = 30.0;
double limelightHeightInches = 10.01;
double goalHeightInches = 60.0;
    //first 6 lines

    public double getDistance(){
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightHeightInches);



        return distanceFromLimelightToGoalInches;
    }
}
