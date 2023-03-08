// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Gripper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunGripperMotorsForSpecificTime extends CommandBase {
  /** Creates a new RunGripperMotorsForSpecificTime. */
  Gripper m_gripper;
  double m_seconds;

  Timer m_timer;

  public RunGripperMotorsForSpecificTime(Gripper gripper, double seconds) {
    m_gripper = gripper;
    m_seconds = seconds;

    m_timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_gripper);
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
    m_gripper.scoreGamePieceCommand();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gripper.runMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(1);
  }
}
