// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class stopArmAndWristCommand extends InstantCommand {
  Arm m_arm;
  Wrist m_wrist;
  public stopArmAndWristCommand(Arm arm, Wrist wrist) {
    m_arm = arm;
    m_wrist = wrist;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm, m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.m_arm_motor.set(ControlMode.PercentOutput, 0);
    m_wrist.m_wrist_motor.set(ControlMode.PercentOutput, 0);
  }
}
