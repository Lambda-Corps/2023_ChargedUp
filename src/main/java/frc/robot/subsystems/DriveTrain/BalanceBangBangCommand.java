// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalanceBangBangCommand extends CommandBase {
  boolean m_done;
  int m_count;
  DriveTrain m_dt;

  /** Creates a new BalanceBangBangCommand. */
  public BalanceBangBangCommand(DriveTrain dt) {
    m_dt = dt;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_done = false;
    m_count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double out = m_dt.drive_bang_bang();
    if( out == 0){
      m_count++;
    } else {
      m_count = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_count > 25; // half second
  }
}
