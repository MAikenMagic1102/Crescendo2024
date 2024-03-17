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
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Vision.Limelight;

public class ScoreControl extends Command {
  private Arm arm;
  private Limelight limelight;
  private boolean isFinished = false;
  /** Creates a new ShooterControl. */
  public ScoreControl(Arm m_arm, Limelight m_limelight){
    arm = m_arm;
    limelight = m_limelight;
    addRequirements(arm);
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
    arm.setArmtoScorePosition(limelight.getLimelightDistance());
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
