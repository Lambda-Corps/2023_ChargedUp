// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WristThenArmSequenceCommand extends SequentialCommandGroup {
  /** Creates a new WristThenArmSequenceCommand. */
  public WristThenArmSequenceCommand(Arm arm, Arm.SuperStructurePosition position_req) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Set the direction for motion magic constants
      arm.configureWristMM( ()->(arm.getSuperStructureWristPosition() < position_req.getWristPosition()) ),
      // Move the wrist to it's spot
      // We want to finish in either 2 seconds, or when we get to the spot.  Whichever hits first
      arm.moveWristToPositionMM(position_req, ()->(arm.getSuperStructureWristPosition() < position_req.getWristPosition()) ).unless(()->arm.isTransitionInvalid(position_req))
    );
  }
}
