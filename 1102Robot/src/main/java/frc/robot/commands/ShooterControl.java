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
import frc.robot.subsystems.Shooter;

public class ShooterControl extends Command {
  private Shooter m_Shooter;
  private SendableChooser<Double> m_SpeedChooser;
  private BooleanSupplier m_feedNow;
  /** Creates a new ShooterControl. */
  public ShooterControl(Shooter shooter, BooleanSupplier feedNow){
    m_feedNow = feedNow;
    m_Shooter = shooter;
    addRequirements(m_Shooter);

    m_SpeedChooser = new SendableChooser<Double>();
    m_SpeedChooser.addOption("100%", 1.0);
    m_SpeedChooser.addOption("90%", 0.9);
    m_SpeedChooser.addOption("80%", 0.8);
    m_SpeedChooser.addOption("70%", 0.7);
    m_SpeedChooser.addOption("60%", 0.6);
    m_SpeedChooser.addOption("50%", 0.5);
    m_SpeedChooser.setDefaultOption("40%", 0.4);
    m_SpeedChooser.addOption("30%", 0.3);
    m_SpeedChooser.addOption("20%", 0.2);

    SmartDashboard.putData("Shooter Throttle", m_SpeedChooser);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_feedNow.getAsBoolean()){
      m_Shooter.feederIn();
    }else{
      m_Shooter.feederStop();
    }

    m_Shooter.setShooterThrottle(m_SpeedChooser.getSelected());
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
