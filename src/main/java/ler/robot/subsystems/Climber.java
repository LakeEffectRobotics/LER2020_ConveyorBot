/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Copied directly from turret bot code

package ler.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * Subsystem representing the climber mech.
 */
public class Climber extends SubsystemBase{
    
    DoubleSolenoid piston;
    TalonFX winch;

    /**
     * Raise the climber.
     * 
     * @param piston The piston attached to the climber
     * @param winch The winch motor
     */
    public Climber(DoubleSolenoid piston, TalonFX winch){
        this.piston = piston;
        this.winch = winch;
    }
    
    /**
     * Raise the climber.
     */
    public void raiseElevator() {
        piston.set(Value.kForward);
    }

    /**
     * Lower the climber.
     */
    public void lowerElevator() {
        piston.set(Value.kReverse);
    }

    /**
     * Winch up the climber.
     * 
     * @param speed Output % to winch with
     */
    public void driveWinch(double speed) {
        winch.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Check if the elevator is extended.
     * 
     * @return <code>true</code> if the climber is extended
     */
    public boolean isElevatorExtended(){
        if(piston.get() == Value.kForward){
            return true;
        }
        return false;
    }
}