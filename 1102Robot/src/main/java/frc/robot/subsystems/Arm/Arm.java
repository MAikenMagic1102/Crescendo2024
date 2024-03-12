// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.Volts;

import org.opencv.core.Mat;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.ScoringTarget;
import frc.robot.ScoringTarget.Position;

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
  private final MotionMagicExpoVoltage mm_ArmPosition = new MotionMagicExpoVoltage(0);

  //private final PositionTorqueCurrentFOC positionTeleTQ = new PositionTorqueCurrentFOC(0);
  private final PositionVoltage postitionTelescope = new PositionVoltage(0);
  private final MotionMagicExpoVoltage mm_telePosition = new MotionMagicExpoVoltage(0);

  private boolean holdingArm = true;
  private boolean holdingTele = true;

  private double currentTargetArmPosition = 0.0;
  private double currentTargetTelescopePosition = 0.0;

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

  private final VoltageOut m_sysidControl = new VoltageOut(0.0);

  private SysIdRoutine m_sysidRoutine = 
    new SysIdRoutine(
      new SysIdRoutine.Config(
        null, 
        Volts.of(4), 
        null,
        (state)->SignalLogger.writeString("state", state.toString())), 
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts)-> m_ArmLeftMotor.setControl(m_sysidControl.withOutput(volts.in(Volts))),
        null, 
        this));


  public Arm() {
    
    m_ArmLeftMotor = new TalonFX(Constants.Arm.leftArmMotorID, Constants.canivoreBus);
    m_ArmRightMotor = new TalonFX(Constants.Arm.rightArmMotorID, Constants.canivoreBus);

    m_ArmRightMotor.setControl(new Follower(Constants.Arm.leftArmMotorID, true));

    m_TelescopeMotor = new TalonFX(Constants.Arm.telescopeMotorID, Constants.canivoreBus);

    TalonFXConfiguration armConfigs = new TalonFXConfiguration();
    
    //PID Configs
    // armConfigs.Slot0.kP = 2000; // An error of 1 rotations results in 40 amps output
    // armConfigs.Slot0.kD = 500; // A change of 1 rotation per second results in 2 amps output
    // // Peak output of 130 amps
    // armConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 2100;
    // armConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -2100;
    armConfigs.Slot0.kS = 0.6;
    armConfigs.Slot0.kV = 0;
    armConfigs.Slot0.kG = 1.52;

    armConfigs.Slot0.kP = 1100;
    armConfigs.Slot0.kD = 30.00;

    armConfigs.MotionMagic.MotionMagicCruiseVelocity = 0.2;


    // //Software Limits
    armConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    armConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    armConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.21;
    armConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.05;

    armConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;



    //Gearing
    armConfigs.Feedback.SensorToMechanismRatio = 133;

    TalonFXConfiguration teleConfigs = new TalonFXConfiguration();

    //PID Configs
    // teleConfigs.Slot0.kP = 30; // An error of 1 rotations results in 40 amps output
    // teleConfigs.Slot0.kD = 5; // A change of 1 rotation per second results in 2 amps output
    // Peak output of 130 amps
    // teleConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 500;
    // teleConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -600;

    teleConfigs.Slot0.kS = 1.1;
    teleConfigs.Slot0.kV = 0;
    teleConfigs.Slot0.kG = 0;

    teleConfigs.Slot0.kP = 150;
    teleConfigs.Slot0.kD = 8;

    teleConfigs.MotionMagic.MotionMagicCruiseVelocity = 0.4;

    //Software Limits
    teleConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    teleConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    teleConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.8;
    teleConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

    teleConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
    teleConfigs.Feedback.SensorToMechanismRatio = 25;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_ArmLeftMotor.getConfigurator().apply(armConfigs);
      status = m_ArmRightMotor.getConfigurator().apply(armConfigs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    //Now do Telescope?
    StatusCode telescopestatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      telescopestatus = m_TelescopeMotor.getConfigurator().apply(teleConfigs);
      if (telescopestatus.isOK()) break;
    }
    if(!telescopestatus.isOK()) {
      System.out.println("Could not apply configs, error code: " + telescopestatus.toString());
    }
    
    // BaseStatusSignal.setUpdateFrequencyForAll(250, 
    // m_ArmLeftMotor.getPosition(),
    // m_ArmLeftMotor.getVelocity(),
    // m_ArmLeftMotor.getMotorVoltage());

    // m_ArmLeftMotor.optimizeBusUtilization();
    //SignalLogger.start();

    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));

    //Startup at 0 position
    m_ArmLeftMotor.setPosition(0.0);
    m_TelescopeMotor.setPosition(0);

    leftSim = m_ArmLeftMotor.getSimState();
    teleSim = m_TelescopeMotor.getSimState();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return m_sysidRoutine.quasistatic(direction);
  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return m_sysidRoutine.dynamic(direction);
  }

  public void setArmMotorOpenLoop(double Armoutput){
    holdingArm = false;
    if(Armoutput < 0.0 && this.getArmPosition() <= 0.0 && !this.isIntakeFullyExtended()){
      this.setArmStop();
    }else{
      leftOut.Output = Armoutput;
      m_ArmLeftMotor.setControl(leftOut);
    }
  }

  public void setTeleMotorOpenLoop(double Teleoutput){
    holdingTele = false;
    telescopeOut.Output = Teleoutput;
    m_TelescopeMotor.setControl(telescopeOut);
  }

  public void setArmPosition(double position){
    holdingArm = true;
    if(position < -0.02 && !this.isIntakeFullyExtended()){
      //Do not try.
      this.setArmStop();
    }else{
      mm_ArmPosition.withPosition(position);
      m_ArmLeftMotor.setControl(mm_ArmPosition);
    }

  }

  public void setArmStop(){
    m_ArmLeftMotor.setControl(new StaticBrake());
  }

  public void setTelescopePosition(double position){
    holdingTele = true;
    postitionTelescope.withPosition(position);
    m_TelescopeMotor.setControl(postitionTelescope);
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
    return getArmPosition() > 0.01 || this.isIntakeFullyExtended();
  }

  public boolean isIntakeRetractSafe(){
        return (getArmPosition() > 0.01 && getArmPosition() < 0.15) || this.isIntakeRetracted();
  }

  public boolean isIntakeRetracted(){
    return getTelescopePosition() < 1.0;
  }

  public boolean isIntakeFullyExtended(){
    return getTelescopePosition() > 1.62;
  }

  public boolean isArmAtAmpPosition(){
    return getArmPosition() > 0.23;
  }

  public boolean isArmAtGroundPosition(){
    return getArmPosition() < -0.03;
  }

  public void setArmtoScorePosition(double distance){
    switch (ScoringTarget.getTarget()) {
      case AMP:
        currentTargetArmPosition = Constants.Arm.AMP.rotArmSetpoint;
        currentTargetTelescopePosition = Constants.Arm.AMP.telescopeSetpoint;
      break;
      case SUBWOOFER:
        currentTargetArmPosition = Constants.Arm.SUBWOOFER.rotArmSetpoint;
        currentTargetTelescopePosition = Constants.Arm.SUBWOOFER.telescopeSetpoint;
      break;
      case RANGED:
      //Lookup Range in a TreeMap or 2?
        //currentTargetArmPosition = Constants.Arm.armMap.get(distance);
      break;
    }
    setArmPosition(currentTargetArmPosition);
    setTelescopePosition(currentTargetTelescopePosition);
  }

  @Override
  public void periodic() {
    // //This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Motor Output", m_ArmLeftMotor.get());
    SmartDashboard.putNumber("Telescope Motor Output", m_TelescopeMotor.get());

    SmartDashboard.putNumber("Arm Position", m_ArmLeftMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Telescope Position", m_TelescopeMotor.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("Arm Setpoint", this.getArmSetpoint());
    SmartDashboard.putNumber("Telescope Setpoint", this.getTelescopeSetpoint());

    SmartDashboard.putString("Arm Target Score Position", ScoringTarget.getTarget().toString());

    SmartDashboard.putNumber("Arm Voltage", m_ArmLeftMotor.getMotorVoltage().getValue());
    SmartDashboard.putNumber("Telescope Voltage", m_TelescopeMotor.getMotorVoltage().getValue());

    SmartDashboard.putNumber("Arm Velocity", m_ArmLeftMotor.getVelocity().getValue());
    SmartDashboard.putNumber("Telescope Veloctiy", m_TelescopeMotor.getVelocity().getValueAsDouble());
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
