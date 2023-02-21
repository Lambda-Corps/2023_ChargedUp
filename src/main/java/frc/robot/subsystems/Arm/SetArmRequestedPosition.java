// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm.Arm.SuperStructurePosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmRequestedPosition extends InstantCommand {
  Arm m_arm;
  SuperStructurePosition m_requested_position;
  public SetArmRequestedPosition(Arm arm, SuperStructurePosition requested_position) {
    m_arm = arm;
    m_requested_position = requested_position;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.requestArmMove(m_requested_position);
  }
}
