// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Gripper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ScoreCone extends CommandBase {
  /** Creates a new ScoreCone. */
  Gripper m_grip;
  Timer m_timer;

  public ScoreCone(Gripper grip) {
    m_grip = grip;
    m_timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_grip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_grip.full_speed_eject();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_grip.stop_motors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(1);
  }
}
