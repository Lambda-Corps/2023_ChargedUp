// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import javax.swing.text.html.parser.DTD;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain.DriveMotionMagic;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Gripper.Gripper;
import frc.robot.subsystems.Gripper.RunMotorsBackward;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Pos3ScoreMove extends SequentialCommandGroup {
  /** Creates a new Pos1ScoreMove. */
  public Pos3ScoreMove(DriveTrain dt, Gripper gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunMotorsBackward(gripper).withTimeout(1),
      new DriveMotionMagic(dt, -140)
    );
  }
}
