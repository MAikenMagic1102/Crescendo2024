// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeIndex extends SequentialCommandGroup {
  /** Creates a new IntakeIndex. */
  public IntakeIndex(Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(shooter);
    addCommands(
      new InstantCommand(() -> shooter.feederIndex()),
      new WaitCommand(0.1),
      new InstantCommand(() -> shooter.setShooterThrottle(-0.05 )),
      new InstantCommand(() -> shooter.feederOutSlow()),
      new WaitCommand(0.1),
      new InstantCommand(() -> shooter.ShooterStop()),
      new InstantCommand(() -> shooter.feederStop())
    );
  }
}
