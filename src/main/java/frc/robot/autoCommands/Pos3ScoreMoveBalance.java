// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmThenWristSequenceCommand;
import frc.robot.subsystems.Arm.WristThenArmSequenceCommand;
import frc.robot.subsystems.Arm.Arm.SuperStructurePosition;
import frc.robot.subsystems.DriveTrain.DriveMotionMagic;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.DriveTrain.TurnToAngleWithGyroPID;
import frc.robot.subsystems.Gripper.Gripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Pos3ScoreMoveBalance extends SequentialCommandGroup {
  /** Creates a new Pos1ScoreMoveBalance. */
  public Pos3ScoreMoveBalance(DriveTrain dt, Arm arm, Gripper gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Set our odometry to the starting position
      //dt.setRobotStartingPose(1.89, 4.97, 180),
      // Set the superstructure to the scoring position
      new WristThenArmSequenceCommand(arm, SuperStructurePosition.ScoreConeMid).raceWith(new WaitCommand(3)),
      // TODO test these new commands and remove the prints
      new PrintCommand("Wrist the arm done"),
      // Drop the cone on the peg
      gripper.expandGripperCommand(),
      new PrintCommand("Expand Gripper done"),
      // Stow the arm back in the robot
      new ArmThenWristSequenceCommand(arm, SuperStructurePosition.Stowed).raceWith(new WaitCommand(3)),
      new PrintCommand("ArmThenWrist Done"),
      // Drive backward 100 inches
      new DriveMotionMagic(dt, -150),
      new TurnToAngleWithGyroPID(dt, -90),
      new DriveMotionMagic(dt, 75),
      new TurnToAngleWithGyroPID(dt, 90),
      // This needs to be drive and Bang Bang
      new DriveMotionMagic(dt, 70)
    );
      // dt.driveMotionMagic(-100).until(dt::is_drive_mm_done).andThen(dt.stopMotorsCommand())      );
  }
}
