// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import frc.robot.subsystems.DriveTrain.DriveTrain;

/** Add your docs here. */
class DriveTrainTest {
    static final double DELTA = 1e-2; // acceptable deviation range
    DriveTrain dt;

    TalonFXSimCollection m_left_motors, m_right_motors;
    DifferentialDrivetrainSim dt_sim;
    @BeforeEach // this method will run before each test
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        dt = new DriveTrain();

        m_left_motors = dt.getLeftSimCollection();
        m_right_motors = dt.getRightSimCollection();

       //dt_sim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDoubleFalcon500PerSide, KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);
       dt_sim = dt.getDriveTrainSim();
    }

    @AfterEach // Cleanup any things we need
    void shutdown(){
        // Do stuff if we need
    }

    @Test
    void driveTrainConstructorWorks(){
        assertNotEquals(dt, null);
    }

    @Test
    void driveTrainSimConstructorWorks(){
        assertNotNull(dt_sim);
    }

    @Test
    void teleopDriveFullForward(){
        System.out.println("Pre-Update Left Velocity: " + dt_sim.getLeftVelocityMetersPerSecond());
        System.out.println("Pre-Update Left Velocity: " + dt.getLeftSpeed());
        System.out.println("Pre-Update Left Velocity: " + m_left_motors.getMotorOutputLeadVoltage());
        dt.teleop_drive(1, 0);
        System.out.println("Post TD Left Velocity: " + dt_sim.getLeftVelocityMetersPerSecond());
        System.out.println("Post TD Left Velocity: " + dt.getLeftSpeed());
        System.out.println("Post TD Left Velocity: " + m_left_motors.getMotorOutputLeadVoltage());
        try{
            Thread.sleep(2000);
        } catch(Exception e) {
            System.out.println(e.getMessage());
        }
        dt.simulationPeriodic();
        System.out.println("Post Sleep Left Velocity: " + dt_sim.getLeftVelocityMetersPerSecond());
        System.out.println("Post Sleep Left Velocity: " + dt.getLeftSpeed());
        System.out.println("Post Sleep Left Velocity: " + m_left_motors.getMotorOutputLeadVoltage());

        // Need to find a way to verify that each motor controller is sending
        // the motors in the proper direction
        assertTrue(dt_sim.getLeftVelocityMetersPerSecond() > 0);
        assertTrue(dt.getLeftSpeed() > 0);
        assertTrue(m_left_motors.getMotorOutputLeadVoltage() > 0);
        assertTrue(m_right_motors.getMotorOutputLeadVoltage() > 0);
  
    }
}
