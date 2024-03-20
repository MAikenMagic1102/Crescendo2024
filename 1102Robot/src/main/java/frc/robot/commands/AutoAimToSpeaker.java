// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision.Limelight;

public class AutoAimToSpeaker extends Command {

  private Limelight vision;
  private boolean isFinished = false;
  private SwerveRequest m_Request;
  private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  /** Creates a new AutoAimToSpeaker. */
  public AutoAimToSpeaker(Limelight limelight) {
    vision = limelight;
    addRequirements(vision);
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(vision.isAimedAtSpeaker()){
      isFinished = true; 
    }else{
    m_Request = drive.withVelocityX(0) 
    .withVelocityY(0) 
    .withRotationalRate(vision.aimToTag()); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
