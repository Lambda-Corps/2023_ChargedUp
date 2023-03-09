// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveSlowlyUntilRamp extends CommandBase {
  DriveTrain m_drivetrain;
  /** Creates a new ReplaceMeCommand. */
  public DriveSlowlyUntilRamp(DriveTrain dt) {
    m_drivetrain = dt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.teleop_drive(.3, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted){
      m_drivetrain.teleop_drive(0, 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_drivetrain.isRollGreaterThan3();
    return false;
  }
}
