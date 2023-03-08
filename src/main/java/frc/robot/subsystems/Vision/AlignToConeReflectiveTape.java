// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain.DriveTrain;

public class AlignToConeReflectiveTape extends CommandBase {
  /** Creates a new AlignToConeReflectiveTape. */
  DriveTrain m_dt;
  Vision m_vision;

  CommandXboxController m_driver_controller;

  double m_target_yaw;
  public AlignToConeReflectiveTape(DriveTrain dt, Vision vision, CommandXboxController controller) {
    m_dt = dt;
    m_vision = vision;

    m_driver_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_dt, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_target_yaw = m_vision.get_limelight_target_yaw();
    double forward_speed = m_driver_controller.getRawAxis(4);

    m_vision.turn_on_limelight_LEDs();

    m_dt.align_with_vision(forward_speed, m_target_yaw);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dt.teleop_drive(0, 0);
    m_vision.turn_off_limelight_LEDs();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
