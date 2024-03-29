// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoShooterCommand extends Command {
  private Shooter m_Shooter;
  private boolean isFinished = false;
  /** Creates a new AutoShooterCommand. */
  public AutoShooterCommand(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Shooter = shooter;
    addRequirements(m_Shooter);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Shooter.setShooterSpeed(75);

    if(m_Shooter.getShooterReady()){
      isFinished = true;
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
