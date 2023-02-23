// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ArmDriveToPositionPIDTest extends CommandBase {
  Arm m_arm;
  CommandXboxController m_remote;

  NetworkTableEntry m_kPEntry, m_kdEntry, m_kiEntry, m_kFEntry;
  /** Creates a new DriveToPosition. */
  public ArmDriveToPositionPIDTest(Arm arm, CommandXboxController controller) {
    m_arm = arm;
    m_remote = controller;
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
    // NetworkTable driveTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive Test");
    // m_kPEntry = driveTab.getEntry("Arm kP");
    // m_time_to_velo = driveTab.getEntry("Time to Velo");
    // m_target_velocity = driveTab.getEntry("Target Velocity");
    // m_left_result = driveTab.getEntry("Left Encoder Result");
    // m_right_result = driveTab.getEntry("Right Encoder Result");

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double arm, wrist;
    // arm = m_remote.getRawAxis(PARTNER_LEFT_AXIS);
    // wrist = m_remote.getRawAxis(PARTNER_RIGHT_AXIS);

    // m_arm.drive_manually_by_position(arm, wrist);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
