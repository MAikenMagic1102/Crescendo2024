// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */

  private double heightOfGoal = 57.13;
  private double heightOfRobotCamera = 10;
  private double cameraMountAngle = 30;

  private boolean hasNotAppliedPrio = false;

  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-magic");
  NetworkTableEntry ty = limelightTable.getEntry("ty");


  public Limelight() {
  }

  public double getLimelightDistance(){
    double distance = 0.0;

    double targetOffsetAngle_Vertical = Rotation2d.fromDegrees(30 + ty.getDouble(0.0)).getRadians();
    distance = (heightOfGoal-heightOfRobotCamera) / Math.tan(targetOffsetAngle_Vertical);

    return distance;
  }

  @Override
  public void periodic() {
    try{
      if ((!hasNotAppliedPrio || DriverStation.isDisabled())) {
      if(DriverStation.getAlliance().get().equals(Alliance.Red)){
        LimelightHelpers.setPriorityTagID("limelight-magic", 4);
        hasNotAppliedPrio = true;
      }

      if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
        LimelightHelpers.setPriorityTagID("limelight-magic", 7);
        hasNotAppliedPrio = true;
      }
      }
    }catch(Exception e){
      
    }

  }
}