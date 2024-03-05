// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmToStow extends SequentialCommandGroup {
  /** Creates a new ArmToIntake. */
  public ArmToStow(Arm arm) {
    addRequirements(arm);

    addCommands(
      new ConditionalCommand(
        new InstantCommand(), 
        new InstantCommand(() -> arm.setArmPosition(Constants.Arm.ArmExtendSafe)), 
        arm::isIntakeRetractSafe),
      new WaitUntilCommand(arm::isIntakeRetractSafe),
      new InstantCommand(() -> arm.setTelescopePosition(Constants.Arm.STOW.telescopeSetpoint)),
      new WaitUntilCommand(arm::isIntakeRetracted),
      new InstantCommand(() -> arm.setArmPosition(Constants.Arm.STOW.rotArmSetpoint))

    );
  }
}
