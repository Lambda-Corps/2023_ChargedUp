// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.kEncoderTicksPerDegree;

public class TurnToAngleMM extends CommandBase {
  DriveTrain m_driveTrain;
  double m_arc_length_ticks;

  int STABLE_ITERATIONS_BEFORE_FINISHED = 5;
  int m_count;

  public TurnToAngleMM(DriveTrain driveTrain, double angle) {
    m_driveTrain = driveTrain;
    m_arc_length_ticks = angle * kEncoderTicksPerDegree;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_count = 0;
    m_driveTrain.motionMagicStartConfigsTurn((m_arc_length_ticks < 0), m_arc_length_ticks);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_driveTrain.motionMagicTurn(m_arc_length_ticks)){
      m_count++;
    } else {
      m_count = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.teleop_drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_count >= STABLE_ITERATIONS_BEFORE_FINISHED;
  }
}