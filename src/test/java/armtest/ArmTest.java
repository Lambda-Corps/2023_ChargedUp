// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package armtest;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.subsystems.Arm.Arm;

/** Add your docs here. */
public class ArmTest {
    static final double DELTA = 1e-2; // acceptable deviation range
    static Arm m_arm = new Arm();
    WPI_TalonFX m_bottom_motor, m_upper_motor;
    DigitalInput m_upper_reverse_limit, m_lower_reverse_limit;
    DIOSim sim_bottom_limit, sim_upper_limit;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);

        m_bottom_motor = m_arm.getBottomStageMotor();
        m_upper_motor = m_arm.getTopStageMotor();

        m_lower_reverse_limit = m_arm.getBottomLimitSwitch();
        m_upper_reverse_limit = m_arm.getTopLimitSwitch();
        sim_bottom_limit = new DIOSim(m_lower_reverse_limit);
        sim_upper_limit = new DIOSim(m_upper_reverse_limit);
    }

    @AfterEach
    void shutdown() {
        // Clean up whatever on shutdown
    }

    @Test
    void armConstructs() {
        assertNotEquals(m_arm, null);

        // assertTrue(m_arm.getArmState() == Arm.ArmState.Inactive);
        // assertTrue(m_arm.getArmControlMode() == Arm.ArmControlMode.Automatic);
        // assertTrue(m_arm.getArmTask() == Arm.ArmTask.Stop);
    }

    @Test 
    void testManualMoveArmZero() {
        double upper_speed = 0;
        double lower_speed = 0;
        sim_bottom_limit.setValue(false);
        sim_upper_limit.setValue(false);
        m_arm.moveArmManually(upper_speed, lower_speed);

        waitForUpdate();

        assertEquals(0, m_bottom_motor.getSelectedSensorVelocity());
        assertEquals(0, m_upper_motor.getSelectedSensorVelocity());
    }

    @Test 
    void testManualMoveArmPositive() {
        double upper_speed = 1.0;
        double lower_speed = 1.0;
        double busV = 12;
        sim_bottom_limit.setValue(false);
        sim_upper_limit.setValue(false);
        m_arm.moveArmManually(upper_speed, lower_speed);

        var lower_sim = m_bottom_motor.getSimCollection();
        var upper_sim = m_upper_motor.getSimCollection();
        lower_sim.setBusVoltage(busV);
        upper_sim.setBusVoltage(busV);

        waitForUpdate();

        // assertTrue(lower_sim.getMotorOutputLeadVoltage() > 0);
        // assertTrue(upper_sim.getMotorOutputLeadVoltage() > 0);
    }

    @Test 
    void testManualMoveArmNegative() {
        double upper_speed = -1.0;
        double lower_speed = -1.0;
        double busV = 12;
        sim_bottom_limit.setValue(false);
        sim_upper_limit.setValue(false);
        m_arm.drive_manually(upper_speed, lower_speed);

        var lower_sim = m_bottom_motor.getSimCollection();
        var upper_sim = m_upper_motor.getSimCollection();
        lower_sim.setBusVoltage(busV);
        upper_sim.setBusVoltage(busV);
        
        for( var i = 0; i < 10; i++){
            waitForUpdate();
        }

        System.out.println("Lower Voltage: " + lower_sim.getMotorOutputLeadVoltage());
        System.out.println("Upper Voltage: " + upper_sim.getMotorOutputLeadVoltage());

        // assertTrue(lower_sim.getMotorOutputLeadVoltage() < 0);
        // assertTrue(upper_sim.getMotorOutputLeadVoltage() < 0);
    }
    
    @Test
    void testArmReverseLimits() {
        double busV = 12;
        var lower_sim = m_bottom_motor.getSimCollection();
        var upper_sim = m_upper_motor.getSimCollection();
        lower_sim.setBusVoltage(busV);
        upper_sim.setBusVoltage(busV);

        // // Test both arm limits false
        // sim_bottom_limit.setValue(false);
        // sim_upper_limit.setValue(false);
        // m_arm.moveArmManually(lower_speed, upper_speed);
        // waitForUpdate();
        // assertTrue(lower_sim.getMotorOutputLeadVoltage() < 0);
        // assertTrue(upper_sim.getMotorOutputLeadVoltage() < 0);

        // // Test lower arm limit is hit
        // sim_bottom_limit.setValue(true);
        // sim_upper_limit.setValue(false);
        // m_arm.moveArmManually(lower_speed, upper_speed);
        // waitForUpdate();
        // assertTrue(lower_sim.getMotorOutputLeadVoltage() == 0);
        // assertTrue(upper_sim.getMotorOutputLeadVoltage() < 0);

        // // Test upper arm limit is hit
        // sim_bottom_limit.setValue(false);
        // sim_upper_limit.setValue(true);
        // m_arm.moveArmManually(lower_speed, upper_speed);
        // waitForUpdate();
        // assertTrue(lower_sim.getMotorOutputLeadVoltage() < 0);
        // assertTrue(upper_sim.getMotorOutputLeadVoltage() == 0);

        // // Test both limits are hit
        // sim_bottom_limit.setValue(true);
        // sim_upper_limit.setValue(true);
        // m_arm.moveArmManually(lower_speed, upper_speed);
        // waitForUpdate();
        // assertTrue(lower_sim.getMotorOutputLeadVoltage() == 0);
        // assertTrue(upper_sim.getMotorOutputLeadVoltage() == 0);
    }

    private static void waitForUpdate() {
        try {
            com.ctre.phoenix.unmanaged.Unmanaged.feedEnable(500);
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
