// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist.Wrist;

public class ArmResetPositon extends CommandBase {
  Arm m_arm;
  Wrist m_wrist;
  /** Creates a new ArmResetPositon. */
  public ArmResetPositon(Arm arm, Wrist wrist) {
    m_arm = arm;
    m_wrist = wrist;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setArmEncoderToZero();
    m_wrist.setWristEncoderToZero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
