// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Vision.Vision;

public class AlignToConeTapeWithVision extends CommandBase {
  DriveTrain m_dt;
  Vision m_vision;
  CommandXboxController m_driver_controller;

  double m_tx;

  /** Creates a new AlignWithVision. */
  public AlignToConeTapeWithVision(DriveTrain dt, Vision vision, CommandXboxController driver_controller) {
    m_dt = dt;
    m_vision = vision;
    m_driver_controller = driver_controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_dt, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vision.setLimelightPipeline(Vision.LIME_CONE_PIPE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_tx = m_vision.getLimelightTX();

    PIDController turn_controller = m_dt.get_dt_turn_pidcontroller();
    double turn = turn_controller.calculate(m_tx, 0);

    m_dt.teleop_drive(m_driver_controller.getRawAxis(4), turn);
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
