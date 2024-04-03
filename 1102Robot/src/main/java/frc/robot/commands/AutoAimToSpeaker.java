// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Limelight;

public class AutoAimToSpeaker extends Command {

  private double MaxAngularRate = 4 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private Limelight vision;
  private CommandSwerveDrivetrain m_swerve;

  private boolean isFinished = false;
  private SwerveRequest m_Request;
  private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  /** Creates a new AutoAimToSpeaker. */
  public AutoAimToSpeaker(Limelight limelight, CommandSwerveDrivetrain swerve) {
    vision = limelight;
    m_swerve = swerve;
    addRequirements(vision,m_swerve);
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
      m_Request = drive.withVelocityX(0) 
      .withVelocityY(0) 
      .withRotationalRate(0); 
      m_swerve.setControl(m_Request);
      isFinished = true; 
    }else{
      m_Request = drive.withVelocityX(0) 
      .withVelocityY(0) 
      .withRotationalRate(vision.aimToTag() * MaxAngularRate); 
    }
    m_swerve.setControl(m_Request);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_Request = drive.withVelocityX(0) 
      .withVelocityY(0) 
      .withRotationalRate(0); 
      m_swerve.setControl(m_Request);    
    isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
