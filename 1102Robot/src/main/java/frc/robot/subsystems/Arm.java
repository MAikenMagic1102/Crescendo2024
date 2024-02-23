// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  private TalonFX m_ArmLeftMotor;
  private TalonFX m_ArmRightMotor;

  private TalonFX m_TelescopeMotor;

  private final TalonFXSimState leftSim;

  private final TalonFXSimState teleSim;

  private final DCMotor m_armGearbox = DCMotor.getFalcon500(1);
  private final DCMotor m_elevatorGearbox = DCMotor.getFalcon500(1);

  private final DutyCycleOut leftOut = new DutyCycleOut(0);
  private final DutyCycleOut telescopeOut = new DutyCycleOut(0);

  private final PositionTorqueCurrentFOC positionArmTQ = new PositionTorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTeleTQ = new PositionTorqueCurrentFOC(0);

  private boolean holdingArm = true;
  private boolean holdingTele = true;

  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          m_armGearbox,
          125.0, //Arm Reduction
          SingleJointedArmSim.estimateMOI(Constants.Arm.kArmLength, Constants.Arm.kArmMass),
          Units.inchesToMeters(36),
          Units.rotationsToRadians(-0.025),
          Units.rotationsToRadians(0.3),
          true,
          0.0
          );

  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          15.0,
          Units.lbsToKilograms(15),
          Units.inchesToMeters(0.895),
          Units.inchesToMeters(0),
          Units.inchesToMeters(14),
          false,
          Units.inchesToMeters(0));       

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm and a telescope.
  private final Mechanism2d m_mech2d = new Mechanism2d(2,2 );
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 0.75, 0.25);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 0.25, -90));
  private final MechanismLigament2d m_arm =
    m_armPivot.append(
      new MechanismLigament2d(
          "Arm",
          0.3,
          Units.radiansToDegrees(m_armSim.getAngleRads()),
          12,
          new Color8Bit(Color.kYellow)));

  private final MechanismLigament2d m_telescope =
    m_arm.append(
      new MechanismLigament2d(
          "Telescope",
          m_elevatorSim.getPositionMeters(),
          0,
          6,
          new Color8Bit(Color.kRed)));            


  public Arm() {
    
    m_ArmLeftMotor = new TalonFX(Constants.Arm.leftArmMotorID);
    m_ArmRightMotor = new TalonFX(Constants.Arm.rightArmMotorID);

    m_ArmRightMotor.setControl(new Follower(Constants.Arm.leftArmMotorID, true));

    m_TelescopeMotor = new TalonFX(Constants.Arm.telescopeMotorID);

    TalonFXConfiguration armConfigs = new TalonFXConfiguration();
    
    //PID Configs
    armConfigs.Slot0.kP = 1200; // An error of 1 rotations results in 40 amps output
    armConfigs.Slot0.kD = 2; // A change of 1 rotation per second results in 2 amps output
    // Peak output of 130 amps
    armConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 1400;
    armConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -1400;

    //Software Limits
    //configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    //configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    //configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.2;
    //configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

    armConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //Gearing
    armConfigs.Feedback.SensorToMechanismRatio = 125;

    TalonFXConfiguration teleConfigs = new TalonFXConfiguration();

    //PID Configs
    teleConfigs.Slot0.kP = 5500; // An error of 1 rotations results in 40 amps output
    teleConfigs.Slot0.kD = 2; // A change of 1 rotation per second results in 2 amps output
    // Peak output of 130 amps
    teleConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 6000;
    teleConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -2000;

    //Software Limits
    //configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    //configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    //configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.2;
    //configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

    teleConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
    teleConfigs.Feedback.SensorToMechanismRatio = 15;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_ArmLeftMotor.getConfigurator().apply(armConfigs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    //Now do Telescope?
    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_TelescopeMotor.getConfigurator().apply(teleConfigs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
    

    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));

    //Startup at 0 position
    m_ArmLeftMotor.setPosition(0.0);
    m_TelescopeMotor.setPosition(0);

    leftSim = m_ArmLeftMotor.getSimState();
    teleSim = m_TelescopeMotor.getSimState();
  }

  public void setArmMotorOpenLoop(double Armoutput){
    holdingArm = false;
    leftOut.Output = Armoutput;
    m_ArmLeftMotor.setControl(leftOut);
  }

  public void setTeleMotorOpenLoop(double Teleoutput){
    holdingTele = false;
    telescopeOut.Output = Teleoutput;
    m_TelescopeMotor.setControl(telescopeOut);
  }

  public void setArmPosition(double position){
    holdingArm = true;
    positionArmTQ.withPosition(position).withFeedForward(10);
    m_ArmLeftMotor.setControl(positionArmTQ);
  }

  public void setTelescopePosition(double position){
    holdingTele = true;
    positionTeleTQ.withPosition(position);
    m_TelescopeMotor.setControl(positionTeleTQ);
  }

  public double getArmPosition(){
    return m_ArmLeftMotor.getPosition().getValueAsDouble();
  }

  public double getTelescopePosition(){
    return m_TelescopeMotor.getPosition().getValueAsDouble();
  }

  public boolean getArmAtSetpoint(){
    return Math.abs(m_ArmLeftMotor.getClosedLoopError().getValue()) < 0.1;
  }

  public boolean getTelescopeAtSetpoint(){
    return Math.abs(m_TelescopeMotor.getClosedLoopError().getValue()) < 0.01;
  }

  public double getArmSetpoint(){
    return m_ArmLeftMotor.getClosedLoopReference().getValue();
  }

  public double getTelescopeSetpoint(){
    return m_TelescopeMotor.getClosedLoopReference().getValue();
  }

  public boolean getArmHolding(){
    return holdingArm;
  }

  public boolean getTeleHolding(){
    return holdingTele;
  }

  public boolean isIntakeExtendSafe(){
    return getArmPosition() > 0.04;
  }

  public boolean isIntakeFullyRetracted(){
    return getTelescopePosition() < 0.02;
  }

  public boolean isIntakeFullyExtended(){
    return getTelescopePosition() > 0.05;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Motor Output", m_ArmLeftMotor.get());
    SmartDashboard.putNumber("Telescope Motor Output", m_TelescopeMotor.get());

    SmartDashboard.putNumber("Arm Position", m_ArmLeftMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Telescope Position", m_TelescopeMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    teleSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    // This method will be called once per scheduler run during simulation
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(leftSim.getMotorVoltage());
    m_elevatorSim.setInput(teleSim.getMotorVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);
    m_elevatorSim.update(0.020);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    m_telescope.setLength(m_elevatorSim.getPositionMeters());

    leftSim.setRawRotorPosition(Units.radiansToRotations(m_armSim.getAngleRads()) * 125.0);
    teleSim.setRawRotorPosition((m_elevatorSim.getPositionMeters() / (Math.PI * 0.895 * 2)) * 15.0);
  }
}
