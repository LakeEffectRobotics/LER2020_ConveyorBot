package ler.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import ler.robot.Tools;
import ler.robot.subsystems.Drivetrain;
import ler.robot.subsystems.Gyro;

/**
 * Turn the robot autonomously using the gyroscope.
 */
public class GyroTurnCommand extends CommandBase {
	private double targetAngle;
	private boolean finished = false;
	private boolean absolute;
	private Drivetrain drivetrain;
	private Gyro gyro;
	static final double HIGH_SPEED = 0.4;
	static final double LOW_SPEED = 0.20;
	static final double LOW_SPEED_LIMIT = 0.3;
	static final double THRESHOLD_ANGLE = 40;
	static final double TOLERANCE = 15;
	static final double GYRO_MULTIPLIER = 5.475d / 360d;
	boolean clockwise = false;

	/**
	 * Turns to an angle in degrees. Positive is CCW. If absolute is true, it turns
	 * to the angle relative to the angle it was at when enabled.
	 * 
	 * @author Tim
	 * @param drivetrain The drivetrain subsystem
	 * @param gyro The gyro subsystem
	 * @param targetAngle The angle to turn to
	 * @param absolute If <code>true</code>, will turn using {@link Gyro#getAbsoluteAngle()}
	 */
	public GyroTurnCommand(Drivetrain drivetrain, Gyro gyro, double targetAngle, boolean absolute) {
		this.drivetrain = drivetrain;
		this.gyro = gyro;
		addRequirements(this.drivetrain);
		addRequirements(this.gyro);

		this.absolute = absolute;
		this.targetAngle = targetAngle;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		if(!absolute){
			gyro.zero();
			// target_angle -= target_angle * GYRO_MULTIPLIER; // gyro tends to increase in
			// error as it turns, this compensates (somewhat)
			clockwise = targetAngle < 0; // gyro tends to increase in error as it turns, this compensates (somewhat)
		} else {
			targetAngle = Tools.closestEquivalentAngle(gyro, targetAngle);
			//target_angle += (target_angle - Robot.gyro.getAbsoluteAngle()) * GYRO_MULTIPLIER; // gyro tends to increase in error as it turns, this compensates (somewhat)	
			clockwise = gyro.getAbsoluteAngle() > targetAngle;
    	}
    }

	// Called repeatedly when this Command is scheduled to run
	@Override
    public void execute() {
    	double l = 0;
    	double r = 0;
    	double calculatedSpeed = 0;
    	
    	double difference;

    	if(absolute){
    		difference = targetAngle - gyro.getAbsoluteAngle();
    	} else {
    		difference = targetAngle - gyro.getAngle();
    	}
    	//If the angle difference is above THRESHOLD_ANGLE, turn at HIGH_SPEED
    	if(!clockwise && difference > THRESHOLD_ANGLE){
    		calculatedSpeed = HIGH_SPEED;
    	} else if(clockwise && difference < -THRESHOLD_ANGLE){
    		calculatedSpeed = -HIGH_SPEED;
    	} else if(Math.abs(difference) > TOLERANCE && Math.abs(difference) <= THRESHOLD_ANGLE){
			//If the angle difference is below THRESHOLD_ANGLE, turn at LOW SPEED, 
			//decreasing as it approaches the target
			calculatedSpeed = (difference / THRESHOLD_ANGLE) * LOW_SPEED;
    		//If the speed is lower than the allowed limit, set it to the limit
    		if(Math.abs(calculatedSpeed) < LOW_SPEED_LIMIT){
    			calculatedSpeed = LOW_SPEED_LIMIT * Math.signum(calculatedSpeed);
    		}
    	} else {
			//If the angle difference is below the tolerance then the turn is finished and the command exits
			finished = true;
    	}
    	r = calculatedSpeed;
    	l = -calculatedSpeed;
    	drivetrain.tankDrive(l, r);
    }

	// Make this return true when this Command no longer needs to run execute()
	@Override
    public boolean isFinished() {
		// if (isTimedOut()) {
		//		Robot.autonomous_command_group.cancel();
		// }
		//TODO: re-add finished condition
		return false;// finished || isTimedOut();
    }

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
    	drivetrain.tankDrive(0, 0);
    }
}
