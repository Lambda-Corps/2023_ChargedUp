// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.DriveArmManually;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.DriveTrain.DefaultDriveTrainCommand;
import frc.robot.subsystems.DriveTrain.DriveMotionMagic;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.DriveTrain.SetMaxSpeedCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Human interface
  private final CommandXboxController m_driver_controller = new CommandXboxController(0);
  private final CommandXboxController m_partner_controller = new CommandXboxController(1);

  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_drivetrain = new DriveTrain();
  private final Arm m_arm = new  Arm();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    buildDriveTestTab();
    buildArmTestTab();
    // Set subsystem default commands
    m_drivetrain.setDefaultCommand(new DefaultDriveTrainCommand(m_drivetrain, m_driver_controller));    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_partner_controller.rightBumper().whileTrue(new DriveArmManually(m_arm, m_partner_controller));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new PrintCommand("Auto Needs to be Fixed");
  }

  private void buildDriveTestTab() {
    ShuffleboardTab driveTestTab = Shuffleboard.getTab("Drive Test");

    driveTestTab.add("Right Encoder", 0).withPosition(1, 0).withSize(1, 1);
    driveTestTab.add("Left Encoder", 0).withPosition(0, 0).withSize(1, 1);
    driveTestTab.add("Right Speed", 0).withPosition(3, 0).withSize(1,1);
    driveTestTab.add("Left Speed", 0).withPosition(2, 0).withSize(1,1);
    
    // Set the max speed variables
    driveTestTab.add("Max Speed", 0).withPosition(0, 1).withSize(1,1);
    // driveTestTab.add("Reset Max Speed", m_drivetrain.setMaxValue()).withPosition(1, 2).withSize(2, 1);
    // Set the max speed variables
    driveTestTab.add("Set_Max Speed", new SetMaxSpeedCommand(m_drivetrain)).withPosition(1, 1).withSize(2, 1);

    // Motion Magic
    driveTestTab.add("Target Distance", 0).withPosition(1, 3).withSize(1, 1);
    driveTestTab.add("Time to Velo", 0).withPosition(0, 3).withSize(1, 1);
    driveTestTab.add("Target Velocity", 0).withPosition(2, 3).withSize(1,1);
    driveTestTab.add("Left Encoder Result", 0).withPosition(3, 3).withSize(1,1);
    driveTestTab.add("Right Encoder Result", 0).withPosition(4, 3).withSize(1,1);
    driveTestTab.add("MM kP", 0).withPosition(5, 3).withSize(1,1);
    driveTestTab.add("Drive MM", new DriveMotionMagic(m_drivetrain)).withPosition(6,3).withSize(2, 1);
    driveTestTab.addDouble("Left Error", m_drivetrain::getLeftError).withPosition(0,4).withSize(1,1);
    driveTestTab.addDouble("Right Error", m_drivetrain::getRightError).withPosition(1,4).withSize(1,1);


    // driveTestTab.add("Robot Heading", 0).withPosition(0, 0).withSize(2, 2).withWidget(BuiltInWidgets.kGyro);
    
  }

  private void buildArmTestTab() {
    ShuffleboardTab armTestTab = Shuffleboard.getTab("Arm Test");

    armTestTab.add("ArmEncoder", 0).withPosition(0, 0).withSize(1, 1);
    armTestTab.addBoolean("ArmForward", m_arm::getArmForwardLimit).withPosition(1, 0).withSize(1,1);
    armTestTab.addBoolean("ArmReverse", m_arm::getArmReverseLimit).withPosition(2, 0).withSize(1,1);
    
  
    armTestTab.add("WristEncoder", 0).withPosition(0, 1).withSize(1, 1);

    armTestTab.addBoolean("WristForward", m_arm::getWristForwardLimit).withPosition(1, 1).withSize(1,1);
    armTestTab.addBoolean("WristReverse", m_arm::getWristReverseLimit).withPosition(2, 1).withSize(1,1);
    armTestTab.add("Arm Sub", m_arm);
    
    // // Set the max speed variables
    // armTestTab.add("Max Speed", 0).withPosition(0, 2).withSize(1,1);
    // armTestTab.add("Reset Max Speed", m_drivetrain.setMaxValue()).withPosition(1, 2).withSize(2, 1);
    // // Set the max speed variables
    // armTestTab.addDouble("Current Speed", m_drivetrain::get_max_speed).withPosition(0, 3).withSize(1,1);
    // armTestTab.add("Set_Max Speed", new SetMaxSpeedCommand(m_drivetrain)).withPosition(1, 3).withSize(2, 1);

    // armTestTab.add("Robot Heading", 0).withPosition(0, 0).withSize(2, 2).withWidget(BuiltInWidgets.kGyro);
    
  }
}
