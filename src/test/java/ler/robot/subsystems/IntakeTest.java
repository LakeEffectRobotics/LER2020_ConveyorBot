package ler.robot.subsystems;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.junit.Test;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * JUnit tests to ensure functionality of the intake subsystem.
 */
public class IntakeTest {
    
    // private CommandScheduler scheduler = null;

    // @Before
    // public void setup(){

    // }

    /**
     * Ensure that the constructor properly sets up subsystem.
     */
    @Test
    public void testConstructor(){
        // Arrange
        TalonSRX motor = mock(TalonSRX.class);
        DoubleSolenoid piston = mock(DoubleSolenoid.class);

        // Act
        Intake intake = new Intake(motor, piston);

        // Assert
        assertEquals(intake.isExtended(), false);

    }

    /**
     * Ensure that the roller speed is set properly.
     */
    @Test
    public void testSpeed(){
        // Arrange
        TalonSRX motor = mock(TalonSRX.class);
        DoubleSolenoid piston = mock(DoubleSolenoid.class);
        Intake intake = new Intake(motor, piston);

        double speed = 0.5;
        
        // Act
        intake.startIntake(speed);

        if(speed > 1) speed=1;
        if(speed < -1) speed=-1;

        verify(motor).set(ControlMode.PercentOutput, speed);
    }
}