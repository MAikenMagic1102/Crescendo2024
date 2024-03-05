// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;

public class ArmControl extends Command {
  private Arm m_Arm;

  private DoubleSupplier m_armSupplier, m_teleSupplier;

  private double lastArmSetpoint = 0.0;
  private double lastTeleSetpoint = 0.0;

  private double armVal, teleVal;

  /** Creates a new ArmControl. */
  public ArmControl(Arm arm, DoubleSupplier armSupplier, DoubleSupplier teleSupplier) {
    m_Arm = arm;
    m_armSupplier = armSupplier;
    m_teleSupplier = teleSupplier;
    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Checks setpoints from last loop including those set by button functions
    lastArmSetpoint = m_Arm.getArmSetpoint();
    lastTeleSetpoint = m_Arm.getTelescopeSetpoint();

    armVal = MathUtil.applyDeadband(m_armSupplier.getAsDouble(), 0.1);
    teleVal = MathUtil.applyDeadband(m_teleSupplier.getAsDouble(), 0.1);

    if(Math.abs(armVal) > 0.1){ //If we are pushing thumbstick we want manual control
      m_Arm.setArmMotorOpenLoop(armVal * 0.6);
    }else{
      if(!m_Arm.getArmHolding()){ //If we just moved the arm we need to save the current position to hold.
        m_Arm.setArmStop();
        lastArmSetpoint = m_Arm.getArmPosition();
        //If we aren't using the thumbsick we want to hold position
        m_Arm.setArmPosition(lastArmSetpoint);
      }

    }

    if(Math.abs(teleVal) > 0.1){ //If we are pushing thumbstick we want manual control
      m_Arm.setTeleMotorOpenLoop(teleVal * 0.7);
    }else{
      if(!m_Arm.getTeleHolding()){ //If we just moved the telescope we need to save the current position to hold.
        m_Arm.setTeleMotorOpenLoop(0.0 * 0.3);
        lastTeleSetpoint = m_Arm.getTelescopePosition();
        //If we aren't using the thumbsick we want to hold position
        m_Arm.setTelescopePosition(lastTeleSetpoint);
      }

    }

  }


}
