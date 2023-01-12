// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.*;

public class DefaultDriveTrainCommand extends CommandBase {
  private final DriveTrain m_drivetrain;
  private final XboxController m_driver_controller;

  /** Creates a new DefaultDriveTrainCommand. */
  public DefaultDriveTrainCommand(DriveTrain dt, XboxController xbox) {
    m_drivetrain = dt;
    m_driver_controller = xbox;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turn, forward;
    if(Robot.isSimulation()){
        turn = m_driver_controller.getRawAxis(0); // Right X
        forward  = -m_driver_controller.getRawAxis(DRIVER_LEFT_AXIS); // Left Y
    }
    else {
        // Axises are inverted, negate them so positive is forward
        turn = m_driver_controller.getRawAxis(DRIVER_RIGHT_AXIS); // Right X
        forward  = -m_driver_controller.getRawAxis(DRIVER_LEFT_AXIS); // Left Y
    }

    if (m_driver_controller.getRightBumper()) {
        // flip controls activated
        forward = -forward;
    }

    m_drivetrain.teleop_drive(forward, turn);
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
