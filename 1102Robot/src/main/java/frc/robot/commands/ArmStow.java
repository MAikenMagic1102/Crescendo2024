// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmStow extends Command {

  private Arm m_arm;

  private boolean isFinished;
  /** Creates a new ArmToIntakePos. */
  public ArmStow(Arm arm) {
    m_arm = arm;
    addRequirements(m_arm);
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
    if(m_arm.isIntakeExtendSafe()){
      m_arm.setTelescopePosition(0);

      if(m_arm.isIntakeFullyRetracted()){
        m_arm.setArmPosition(0.0);
        isFinished = true;
      }
    }else{
      m_arm.setArmPosition(0.05);
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
