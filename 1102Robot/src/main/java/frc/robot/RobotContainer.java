// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.Utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Vision.Limelight;
import frc.robot.ScoringTarget.Position;
import frc.robot.commands.*;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
    private final CommandXboxController joystick2 = new CommandXboxController(1); // My joystick
  
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final CANdleSystem candle = new CANdleSystem();

  private final Shooter shooter = new Shooter(candle);

  private final Arm arm = new Arm();

  Limelight vision = new Limelight();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final SendableChooser<Command> autoChooser;  
  
  public RobotContainer() {
    configureBindings();

    NamedCommands.registerCommand("armToScore", new Score(arm));
    NamedCommands.registerCommand("intakeSetpoint", new ArmToIntake(arm));
    NamedCommands.registerCommand("stowArm", new ArmToStow(arm));
    NamedCommands.registerCommand("getShooterSpunUp", new AutoShooterCommand(shooter));
    NamedCommands.registerCommand("feederNoteIn", new IntakeNote(shooter));
    NamedCommands.registerCommand("shootNote", new InstantCommand(() -> shooter.feederShootNow()));
    NamedCommands.registerCommand("feederStop", new InstantCommand(() -> shooter.feederStop()));
    NamedCommands.registerCommand("shooterStop", new InstantCommand(() -> shooter.ShooterStop()));
    NamedCommands.registerCommand("setPodium", new InstantCommand(() -> ScoringTarget.setTarget(Position.PODIUM)));
    NamedCommands.registerCommand("indexNote", new IntakeIndex(shooter));

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("No Auto", null);
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(
      new SwerveDriveControl(
        drivetrain, 
        vision,
        () -> -joystick.getLeftX(),  //Translation 
        () -> -joystick.getLeftY(),  //Translation
        () -> -joystick.getRightX(), //Rotation
        joystick.rightStick(), //Limelight Aim to Goal
        joystick.povDown(), 
        joystick.y(), //Face Forward
        joystick.b(), //Face Right
        joystick.a(), //Face Backwards
        joystick.x()  //Face Left
      )
    );

    //shooter.setDefaultCommand(new FeederControl(shooter, joystick.leftBumper(), joystick.leftTrigger()));

    arm.setDefaultCommand(new ArmControl(arm, () -> -joystick2.getRightY(), () -> -joystick2.getLeftY()));

    joystick2.a().onTrue(new InstantCommand(() -> ScoringTarget.setTarget(Position.SUBWOOFER)).andThen(new InstantCommand(() -> candle.setGreen())));
    joystick2.b().onTrue(new InstantCommand(() -> ScoringTarget.setTarget(Position.AMP)).andThen(new InstantCommand(() -> candle.setBlue())));
    joystick2.y().onTrue(new InstantCommand(() -> ScoringTarget.setTarget(Position.RANGED)).andThen(new InstantCommand(() -> candle.setRed())));
    joystick2.x().onTrue(new InstantCommand(() -> ScoringTarget.setTarget(Position.PODIUM)).andThen(new InstantCommand(() -> candle.setPurple())));
      // joystick2.a().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
      // joystick2.b().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      // joystick2.x().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      // joystick2.y().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    joystick2.povUp().onTrue(new InstantCommand(() -> arm.preClimb()));
    joystick2.povDown().onTrue(new InstantCommand(() -> arm.Climb()));

    //joystick.povUp().whileTrue(new InstantCommand(() -> shooter.setShooterSpeed(75)));

    //joystick.leftBumper().whileTrue(new ArmToIntake(arm).andThen(new RunCommand(() -> shooter.feederIn())));
    //joystick.leftBumper().whileTrue(new ArmToIntake(arm).andThen(new RunCommand(() -> shooter.feederIn())));
    joystick.leftBumper().whileTrue(new ArmToIntake(arm).andThen(new IntakeNote(shooter)));

    joystick.leftBumper().onFalse(new IntakeIndex(shooter).andThen(new ArmToStow(arm)));

    joystick.leftTrigger().whileTrue(new Score(arm).andThen(new RunCommand(() -> shooter.feederOut())));
    joystick.leftTrigger().onFalse(new InstantCommand(()-> shooter.feederStop()));

    joystick.rightTrigger().whileTrue(new ShooterControl(shooter, joystick.rightBumper()).alongWith(new ScoreControl(arm, vision)));
    joystick.rightTrigger().onFalse(new InstantCommand(() -> shooter.ShooterStop()).andThen(new InstantCommand(() -> shooter.feederStop())).andThen(new ArmToStow(arm)));

    // reset the field-centric heading on left bumper press
    joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.setFieldRelative()));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);
  } 

  public Command getAutonomousCommand() {
      return autoChooser.getSelected();
   }


}
