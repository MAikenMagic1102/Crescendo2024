// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SwerveDriveControl extends Command {
  private CommandSwerveDrivetrain m_Swerve;
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 4 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private DoubleSupplier xSup;
  private DoubleSupplier ySup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier m_halfSpeed;
  private BooleanSupplier m_quarterSpeed;
  private BooleanSupplier m_90, m_180, m_270, m_0;
  
  private double rotationVal, xVal, yVal;
  private double targetAngle = 0.0;
  private PhoenixPIDController m_thetaController;
  private SendableChooser<Double> m_speedChooser;
  private SwerveRequest m_Request;
  private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  private SwerveRequest.FieldCentricFacingAngle driveAngle = new SwerveRequest.FieldCentricFacingAngle();

  /** Creates a new SwerveDriveControl. */
  public SwerveDriveControl(CommandSwerveDrivetrain swerve, DoubleSupplier xSup, DoubleSupplier ySup, 
                            DoubleSupplier rotationSup, BooleanSupplier halfSpeed, BooleanSupplier quarterSpeed,
                            BooleanSupplier zero, BooleanSupplier ninety, BooleanSupplier oneEighty, 
                            BooleanSupplier twoSeventy) {
    m_Swerve = swerve;
    this.ySup = ySup;
    this.xSup = xSup;
    this.rotationSup = rotationSup;
    m_halfSpeed = halfSpeed;
    m_quarterSpeed = quarterSpeed;
    m_0 = zero;
    m_90 = ninety;
    m_180 = oneEighty;
    m_270 = twoSeventy;
    addRequirements(m_Swerve);
    m_speedChooser = new SendableChooser<Double>();
    m_speedChooser.addOption("100%", 1.0);
    m_speedChooser.addOption("90%", 0.9);
    m_speedChooser.setDefaultOption("85%", 0.85);
    m_speedChooser.addOption("80%", 0.8);
    m_speedChooser.addOption("70%", 0.7);
    m_speedChooser.addOption("60%", 0.6);
    m_speedChooser.addOption("50%", 0.5);
    m_speedChooser.addOption("40%", 0.4);
    m_speedChooser.addOption("30%", 0.3);
    m_speedChooser.addOption("20%", 0.2);
    SmartDashboard.putData("Speed Percent", m_speedChooser);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_thetaController = new PhoenixPIDController(5.0, 0.0, 0.0);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    driveAngle.HeadingController = m_thetaController;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean rotateWithButton = m_0.getAsBoolean() || m_90.getAsBoolean() || m_180.getAsBoolean() || m_270.getAsBoolean();

    xVal = MathUtil.applyDeadband(xSup.getAsDouble() * m_speedChooser.getSelected(), 0.1);
    yVal = MathUtil.applyDeadband(ySup.getAsDouble() * m_speedChooser.getSelected(), 0.1);
    rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble() * m_speedChooser.getSelected(), 0.1);

    if(rotateWithButton){
      if(m_0.getAsBoolean()){
        targetAngle = (0.0);
      }
      else if(m_90.getAsBoolean()){
        targetAngle = (-90.0);
      }
      else if(m_180.getAsBoolean()){
        targetAngle = (180.0);
      }
      else if(m_270.getAsBoolean()){
        targetAngle = (90.0);
      }

      m_Request = driveAngle.withVelocityX(yVal * MaxSpeed)
          .withVelocityY(xVal * MaxSpeed)
          .withTargetDirection(Rotation2d.fromDegrees(targetAngle));
    }else{
      m_Request = drive.withVelocityX(yVal * MaxSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(xVal * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(rotationVal * MaxAngularRate); // Drive counterclockwise with negative X (left)
    }

    m_Swerve.setControl(m_Request);
  }   
}
