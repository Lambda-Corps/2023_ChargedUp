// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.kEncoderTicksPerInch;

import java.io.Console;


public class DriveWithMotionMagic extends CommandBase {
  DriveTrain m_driveTrain;
  ShuffleboardTab driveMMTab;
  double m_targetPosition;
  double m_targetTicks;
  int count;
  double m_start_time;
  double m_drive_kP, m_kI, m_kD, m_kF;
  NetworkTableEntry m_drivekPEntry, m_kIEntry, m_kDEntry, m_kFEntry, m_targetPosEntry, m_targetTicksEntry, m_iterationEntry, m_drivedurationEntry;
  //the number of times motion magic is on target before the command finishes
  int STABLE_ITERATIONS_BEFORE_FINISHED = 5;

  public DriveWithMotionMagic(DriveTrain driveTrain, double targetInches) {
    m_driveTrain = driveTrain;
    m_targetPosition = targetInches;
    NetworkTable driveTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive Test");

    m_drivekPEntry = driveTab.getEntry("kP");
    m_kIEntry = driveTab.getEntry("kI");
    m_kDEntry = driveTab.getEntry("kD");
    m_kFEntry = driveTab.getEntry("kF");
    m_iterationEntry = driveTab.getEntry("Finish Iterations");
    m_targetPosEntry = driveTab.getEntry("Tgt. Inches");
    m_targetTicksEntry = driveTab.getEntry("Tgt. Ticks");
    m_drivedurationEntry = driveTab.getEntry("Run Time");
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_kF = m_kFEntry.getDouble(0);
    m_drive_kP = m_drivekPEntry.getDouble(0.0);
    m_kI = m_kIEntry.getDouble(0.0);
    m_kD = m_kDEntry.getDouble(0.0);
    STABLE_ITERATIONS_BEFORE_FINISHED = (int) m_iterationEntry.getDouble(5.0);
    m_targetTicks = m_targetPosEntry.getDouble(0) * kEncoderTicksPerInch;
    m_targetTicksEntry.setDouble((int) m_targetTicks);
    count = 0;
    m_driveTrain.reset_drive_PID_values(m_drive_kP, m_kI, m_kD);
    m_driveTrain.motion_magic_start_config_drive(m_targetTicks >= 0, m_targetTicks);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_driveTrain.driveMotionMagic((int)m_targetTicks)){
      count++;
      System.out.print("count:" + count);
    } else {
      count = 0;
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
    return count >= STABLE_ITERATIONS_BEFORE_FINISHED;
  }
}
