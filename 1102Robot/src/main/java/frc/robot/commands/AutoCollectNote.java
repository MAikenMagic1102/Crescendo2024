// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.IntakeNote;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class AutoCollectNote extends ParallelRaceGroup {

    CommandSwerveDrivetrain m_drivetrain;
    Limelight m_limelight;
    Shooter m_shooter;

    /** Creates a new autoCollectNote. */
    public AutoCollectNote(CommandSwerveDrivetrain drivetrain, Shooter shooter,
            Limelight limelight) {
        m_drivetrain = drivetrain;
        m_shooter = shooter;
        m_limelight = limelight;
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(new AutoRunToNote(m_drivetrain, m_limelight).withTimeout(2.00));
        addCommands(new IntakeNote(m_shooter));
    }
}
