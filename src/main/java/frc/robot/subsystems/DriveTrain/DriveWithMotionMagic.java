// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveWithMotionMagic extends CommandBase {
  /** Creates a new DriveWithMotionMagic. */
  DriveTrain m_drive_train;
  private double m_target_distance;

  public DriveWithMotionMagic(DriveTrain driveTrain, double target_distance) {
    m_drive_train = driveTrain;
    m_target_distance = target_distance;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive_train);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
