// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Gripper;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Gripper extends SubsystemBase {
  DoubleSolenoid m_gripper;
  
  final DoubleSolenoid.Value GRIPPER_CONTRACT = DoubleSolenoid.Value.kForward;
  final DoubleSolenoid.Value GRIPPER_EXPAND = DoubleSolenoid.Value.kReverse;
  /** Creates a new Gripper. */
  public Gripper() {
    m_gripper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GRIPPER_SOLENOID_CHANNEL_A,
    GRIPPER_SOLENOID_CHANNEL_B);
    // Set the gripper to contracted for our preload
    m_gripper.set(GRIPPER_CONTRACT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public CommandBase expandGripperCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_gripper.set(GRIPPER_EXPAND);
        });
  }

  public CommandBase contractGripperCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_gripper.set(GRIPPER_CONTRACT);
        });
  }

}
