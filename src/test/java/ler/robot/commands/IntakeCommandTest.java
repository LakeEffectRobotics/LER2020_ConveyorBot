package ler.robot.commands;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import ca.gregk.frcmocks.ctre.MockTalonSRX;
import ca.gregk.frcmocks.rev.MockCANSparkMax;
import ca.gregk.frcmocks.scheduler.MockButton;
import ca.gregk.frcmocks.scheduler.MockHardwareExtension;
import ca.gregk.frcmocks.scheduler.TestWithScheduler;
import ca.gregk.frcmocks.wpilib.MockDoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import ler.robot.subsystems.Conveyor;
import ler.robot.subsystems.Intake;

/**
 * JUnit tests to ensure functionality of IntakeCommand and connected subsystems.
 */
public class IntakeCommandTest {
    
    private MockTalonSRX intakeMotor;
    private MockDoubleSolenoid intakePiston;
    private Intake intake;

    private MockTalonSRX conveyorDriveMotor;
    private MockCANSparkMax conveyorAngleMotor;
    private MockDoubleSolenoid conveyorValve;
    private Conveyor conveyor;

    private IntakeCommand command;

    static final double SPEED_TOLERANCE = 0.01;

    /**
     * Setup the scheduler and hardware for simulated testing.
     */
    @Before
    public void before() {
        // Arrange
        intakeMotor = new MockTalonSRX();
        intakePiston = new MockDoubleSolenoid();
        intake = new Intake(intakeMotor.getMock(), intakePiston.getMock());

        conveyorDriveMotor = new MockTalonSRX();
        conveyorAngleMotor = new MockCANSparkMax();
        conveyorValve = new MockDoubleSolenoid();
        conveyor = new Conveyor(conveyorDriveMotor.getMock(), conveyorAngleMotor.getMock(), conveyorValve.getMock());

        command = new IntakeCommand(intake, conveyor);

        TestWithScheduler.schedulerStart();
        TestWithScheduler.schedulerClear();
        MockHardwareExtension.beforeAll();
    }

    /**
     * Ensure that the roller spins and intake deploys on start.
     */
    @Test
    public void startIntake() {
        // Arrange
        MockButton button = new MockButton();
        button.whenPressed(command);
        
        // Act
        button.push();
        CommandScheduler.getInstance().run();
        button.release();

        // Assert
        assertEquals(intakeMotor.controlMode, ControlMode.PercentOutput);
        assertEquals(Math.abs(intakeMotor.setpoint), Math.abs(Intake.ROLLER_SPEED), SPEED_TOLERANCE);
        assertTrue("Intake not extended", intake.isExtended());

        TestWithScheduler.schedulerClear();
    }

    /**
     * Ensure that the roller stops and intake retracts on stop.
     */
    @Test
    public void stopIntake() {
        // Arrange
        MockButton button = new MockButton();
        button.whileHeld(command);
        
        // Act
        // Deploy
        button.push();
        CommandScheduler.getInstance().run();
        //Retract
        button.release();
        CommandScheduler.getInstance().run();

        // Assert
        assertEquals(Math.abs(intakeMotor.setpoint), 0, SPEED_TOLERANCE);
        assertFalse("Intake not retracted", intake.isExtended());

        TestWithScheduler.schedulerClear();
    }
    
    /**
     * Clean up scheduler system.
     */
    @After
    public void after() {
        TestWithScheduler.schedulerDestroy();
        MockHardwareExtension.afterAll();
    }
}