// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.Arm;

public class ArmToAmpPosition extends Command {

  private Arm intakeArm;
  private boolean isFinished = false;

  /** Creates a new ArmToAmpPosition. */
  public ArmToAmpPosition(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeArm = arm;
    addRequirements(intakeArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   

    if(intakeArm.isArmAtAmpPosition()){
      intakeArm.setTelescopePosition(Constants.Arm.AMP.telescopeSetpoint);
      isFinished = true;
    } else {
      intakeArm.setArmPosition(Constants.Arm.AMP.rotArmSetpoint);
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
