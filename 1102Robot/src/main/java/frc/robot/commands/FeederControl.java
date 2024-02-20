// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class FeederControl extends Command {
  private Shooter m_Shooter;
  private BooleanSupplier m_In;
  private BooleanSupplier m_Out;
  /** Creates a new FeederControl. */
  public FeederControl(Shooter shooter, BooleanSupplier in, BooleanSupplier out) {
    m_Shooter = shooter;
    m_In = in;
    m_Out = out;
    addRequirements(m_Shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_In.getAsBoolean()){
      m_Shooter.feederIn();
    }else{

      if(m_Out.getAsBoolean()){
        m_Shooter.feederOut();
      }else{
        m_Shooter.feederStop();
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
