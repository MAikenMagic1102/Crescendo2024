// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.Limelight;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class Shooter extends SubsystemBase {

  private TalonFX Shooter1, Shooter2;
  private TalonFX Feeder;
  private DigitalInput noteSensor;
  private PhoenixPIDController shooterPID;

  private VelocityVoltage shooterSpeed;

  private final VoltageOut m_sysidControl = new VoltageOut(0.0);

  private SysIdRoutine m_sysidRoutine = 
    new SysIdRoutine(
      new SysIdRoutine.Config(
        Volts.of(0.5).per(Second), 
        Volts.of(7), 
        Seconds.of(15),
        (state)->SignalLogger.writeString("feederState", state.toString())), 
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts)-> 
          Feeder.setControl(m_sysidControl.withOutput(volts.in(Volts))),
        null, 
        this));
  
  /** Creates a new Shooter. */
  public Shooter(){

    Shooter1 = new TalonFX(Constants.Shooter.Shooter1_ID,Constants.canivoreBus);
    Shooter2 = new TalonFX(Constants.Shooter.Shooter2_ID,Constants.canivoreBus);

    Feeder = new TalonFX(Constants.Shooter.Feeder_ID,Constants.canivoreBus);

    noteSensor = new DigitalInput(Constants.Shooter.noteSensor_DIO);
    shooterPID = new PhoenixPIDController(Constants.Shooter.Shooter_kP, Constants.Shooter.Shooter_kI, Constants.Shooter.Shooter_kD);

    Shooter1.setInverted(Constants.Shooter.Shooter1_Inverted);
    Shooter2.setInverted(Constants.Shooter.Shooter2_Inverted);
    Feeder.setInverted(Constants.Shooter.Feeder_Inverted);

    TalonFXConfiguration shooterConfiguration = new TalonFXConfiguration();

    shooterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    shooterConfiguration.Slot0.withKP(Constants.Shooter.kP);
    shooterConfiguration.Slot0.withKS(Constants.Shooter.kS);
    shooterConfiguration.Slot0.withKV(Constants.Shooter.kV);
    shooterConfiguration.Slot0.withKA(Constants.Shooter.kA);

    shooterConfiguration.Feedback.SensorToMechanismRatio = Constants.Shooter.Gear_Ratio;

    TalonFXConfiguration Feederconfig = new TalonFXConfiguration();

    Feederconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = Feeder.getConfigurator().apply(Feederconfig);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    StatusCode shooterStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      shooterStatus = Shooter1.getConfigurator().apply(shooterConfiguration);
      shooterStatus = Shooter2.getConfigurator().apply(shooterConfiguration);
      if (shooterStatus.isOK()) break;
    }
    if(!shooterStatus.isOK()) {
      System.out.println("Could not apply configs, error code: " + shooterStatus.toString());
    }

    // BaseStatusSignal.setUpdateFrequencyForAll(250, 
    // Feeder.getPosition(),
    // Feeder.getVelocity(),
    // Feeder.getMotorVoltage());
    // SignalLogger.start();


    Shooter2.setControl(new Follower(Constants.Shooter.Shooter1_ID, true));
    shooterSpeed = new VelocityVoltage(0);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return m_sysidRoutine.quasistatic(direction);
  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return m_sysidRoutine.dynamic(direction);
  }


  public void feederIn(){
    if(!noteSensor.get()){
      Feeder.set(0.78);
    }else{
      Feeder.set(0);
    }
  }

  public void feederOut(){
    Feeder.set(-0.5);
  }

    public void feederOutSlow(){
    Feeder.set(-0.1);
  }


  public void feederStop(){
    Feeder.setControl(new StaticBrake());
  }

  public void feederShoot(boolean ready){
    if(ready){
      Feeder.set(0.8);
    }
    else{
      Feeder.set(0.0);
    }
  }

    public void feederShootNow(){
      Feeder.set(0.8);
    }

    public void feederIndex(){
      Feeder.set(0.3);
    }


  public void ShooterStop(){
    Shooter1.setControl(new StaticBrake());
  }

  public void setShooterThrottle(double throttle){
    Shooter1.set(throttle);
  }

  public void setShooterSpeed(double speed){
    Shooter1.setControl(shooterSpeed.withVelocity(speed));
  } 
  
  public void setShooterDistance(double distance){
    
  }

  public boolean getShooterReady(){
    return Math.abs(Shooter1.getClosedLoopError().getValueAsDouble()) < 3;
  }

  public boolean getIntakeHasNote(){
    return noteSensor.get();
  }

  

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", Shooter1.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Target", Shooter1.getClosedLoopReference().getValue());
    SmartDashboard.putNumber("Shooter Error", Shooter1.getClosedLoopError().getValue());

    SmartDashboard.putNumber("Feeder Speed", Feeder.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Feeder Target", Feeder.getClosedLoopReference().getValue());
    SmartDashboard.putNumber("Feeder Error", Feeder.getClosedLoopError().getValue());

    SmartDashboard.putBoolean("Shooter Ready", getShooterReady());
    SmartDashboard.putBoolean("Note Senor", noteSensor.get());

    // SmartDashboard.putNumber("Feeder Speed", Feeder.getVelocity().getValueAsDouble());
    if(noteSensor.get()){
      LimelightHelpers.setLEDMode_ForceBlink("limelight-magic");
    }else{
      LimelightHelpers.setLEDMode_ForceOff("limelight-magic");
    }

  }
}
