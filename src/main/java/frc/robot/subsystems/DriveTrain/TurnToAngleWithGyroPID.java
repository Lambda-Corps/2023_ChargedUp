// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToAngleWithGyroPID extends CommandBase {
  DriveTrain m_dt;
  double m_target_degrees;
  boolean m_done;
  int m_count_done;

  /** Creates a new TurnToAngleWithGyro. */
  public TurnToAngleWithGyroPID(DriveTrain dt, double target_degrees) {
    m_target_degrees = target_degrees;
    m_dt = dt;
    addRequirements(m_dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_done = false;
    m_count_done = 0;

    m_dt.set_turn_target_setpoint(m_target_degrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (m_dt.turn_target_degrees()) {
    //   m_count_done++;
    // } else {
    //   m_count_done = 0;
    // }
    m_done = m_dt.turn_target_degrees();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dt.teleop_drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_count_done > 5;
    return m_done;
  }
}
