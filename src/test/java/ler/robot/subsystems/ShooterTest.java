package ler.robot.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.revrobotics.ControlType;

import org.junit.Before;
import org.junit.Test;

import ca.gregk.frcmocks.rev.MockCANSparkMax;

/**
 * JUnit tests to ensure functionality of the shooter subsystem.
 */
public class ShooterTest {

    private MockCANSparkMax tlSpark;
    private MockCANSparkMax trSpark;
    private MockCANSparkMax brSpark;
    private MockCANSparkMax blSpark;

    private Shooter shooter;

    static final double SPEED_TOLERANCE = 0.01;

    /**
     * Setup the test subsystem.
     */
    @Before
    public void setup(){
        // Arrange
        tlSpark = new MockCANSparkMax();
        trSpark = new MockCANSparkMax();
        brSpark = new MockCANSparkMax();
        blSpark = new MockCANSparkMax();

        shooter = new Shooter(tlSpark.getMock(), trSpark.getMock(), brSpark.getMock(), blSpark.getMock());

    }

    /**
     * Ensure that the wheel speeds are set properly (top at full, bottom at a multiple of that).
     */
    @Test
    public void setSpeed(){
        // Act
        shooter.setSpecificShootersSpeed(Shooter.SHOOTER_TARGET_SPEED);

        // Assert
        assertTrue("TL Spark in wrong mode", tlSpark.pidMode);
        assertEquals(ControlType.kVelocity, tlSpark.controlType);
        assertEquals(Shooter.SHOOTER_TOP_TARGET_SPEED, tlSpark.setpoint, SPEED_TOLERANCE);

        assertTrue("TR Spark in wrong mode", trSpark.pidMode);
        assertEquals(ControlType.kVelocity, trSpark.controlType);
        assertEquals(-Shooter.SHOOTER_TOP_TARGET_SPEED, trSpark.setpoint, SPEED_TOLERANCE);

        assertTrue("BR Spark in wrong mode", brSpark.pidMode);
        assertEquals(ControlType.kVelocity, brSpark.controlType);
        assertEquals(-Shooter.SHOOTER_BOTTOM_TARGET_SPEED, brSpark.setpoint, SPEED_TOLERANCE);

        assertTrue("BL Spark in wrong mode", blSpark.pidMode);
        assertEquals(ControlType.kVelocity, blSpark.controlType);
        assertEquals(Shooter.SHOOTER_BOTTOM_TARGET_SPEED, blSpark.setpoint, SPEED_TOLERANCE);
    }

    /**
     * Ensure that when the shooter wheels are stopped, they coast instead of braking.
     */
    @Test
    public void stopShooter(){        
        // Act
        // Set speeds
        shooter.setSpecificShootersSpeed(Shooter.SHOOTER_TARGET_SPEED);
        // Kill speeds
        shooter.setSpecificShootersSpeed(0);

        // Assert
        assertFalse("TL Spark in wrong mode", tlSpark.pidMode);
        assertEquals(0, tlSpark.setpoint, 0);

        assertFalse("TR Spark in wrong mode", trSpark.pidMode);
        assertEquals(0, trSpark.setpoint, 0);

        assertFalse("BR Spark in wrong mode", brSpark.pidMode);
        assertEquals(0, brSpark.setpoint, 0);

        assertFalse("BL Spark in wrong mode", blSpark.pidMode);
        assertEquals(0, blSpark.setpoint, 0);


    }
}