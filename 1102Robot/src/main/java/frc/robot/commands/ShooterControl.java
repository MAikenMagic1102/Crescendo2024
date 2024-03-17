// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ScoringTarget;
import frc.robot.ScoringTarget.Position;
import frc.robot.subsystems.Shooter;

public class ShooterControl extends Command {
  private Shooter m_Shooter;
  private SendableChooser<Double> m_SpeedChooser;
  private BooleanSupplier m_feedNow;
  private double shooterSpeed = 0.0;
  /** Creates a new ShooterControl. */
  public ShooterControl(Shooter shooter, BooleanSupplier feedNow){
    m_feedNow = feedNow;
    m_Shooter = shooter;
    SmartDashboard.putNumber("Shooter Target Speed", 75.0);
    shooterSpeed = SmartDashboard.getNumber("Shooter Target Speed", 75.0);
    addRequirements(m_Shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_feedNow.getAsBoolean() && m_Shooter.getShooterReady()){
      m_Shooter.feederShootNow();
    }else{
      m_Shooter.feederStop();
    }



    if(ScoringTarget.getTarget() == Position.AMP){
      m_Shooter.setShooterSpeed(40.0);
    }else{
      if(ScoringTarget.getTarget() == Position.SUBWOOFER){
        //m_Shooter.setShooterThrottle(0.75);
      m_Shooter.setShooterSpeed(75.0);
      }else{
        if(ScoringTarget.getTarget() == Position.PODIUM){
          m_Shooter.setShooterSpeed(77.0);
        }else{
          if(ScoringTarget.getTarget() == Position.RANGED){
            shooterSpeed = SmartDashboard.getNumber("Shooter Target Speed", 75.0);
            m_Shooter.setShooterSpeed(shooterSpeed);
          }          
        }
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
