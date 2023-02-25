// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static frc.robot.Constants.*;

public class DriveArmManually extends CommandBase {
  private final Arm m_arm;
  private final CommandXboxController m_partner_controller;

  /** Creates a new DriveArmManually. */
  public DriveArmManually(Arm arm, CommandXboxController xbox) {
    m_arm = arm;
    m_partner_controller = xbox;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double arm, wrist;
    arm = m_partner_controller.getRawAxis(PARTNER_LEFT_AXIS);
    wrist = m_partner_controller.getRawAxis(PARTNER_RIGHT_AXIS);

    m_arm.drive_manually(arm, wrist);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.drive_manually(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
