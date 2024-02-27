// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.Arm;

public class ArmToIntakePosition extends Command {
  /** Creates a new ArmToIntakePosition. */

  private Arm intakeArm;
  private boolean isFinished = false;

  public ArmToIntakePosition(Arm arm) {
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
    if(intakeArm.isIntakeFullyExtended()){
      intakeArm.setArmPosition(Constants.Arm.INTAKE.rotArmSetpoint);
      // if(intakeArm.isArmAtGround()){
        isFinished = true;
      // }
    } else {

      if(intakeArm.isIntakeExtendSafe()){
        intakeArm.setTelescopePosition(Constants.Arm.INTAKE.telescopeSetpoint);
      } else {
        intakeArm.setArmPosition(Constants.Arm.ArmExtendSafe);
      }
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
