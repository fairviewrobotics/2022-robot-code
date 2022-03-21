// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMax.ControlType.*
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants

class WinchSubsystem(winch: CANSparkMax,
                     val lowerLimit: DigitalInput, 
                     val upperLimit: DigitalInput,
                     val debug: Boolean = false) : SubsystemBase() {
    // Target voltage to run the motor at. If non null, motor is run directly at targetVoltage.
    var targetVoltage: Double? = 0.0
    // Target distance to run the motor to with PID control. PID control is only enabled if targetVoltage is null.
    var targetDist = 0.0

    val pid = winch.getPIDController()
    val encoder = winch.getEncoder()

    init {
        winch.setSmartCurrentLimit(Constants.elevatorMaxCurrent.toInt())
        winch.setSecondaryCurrentLimit(Constants.elevatorMaxCurrent)
        winch.setIdleMode(CANSparkMax.IdleMode.kBrake)

        status = "OK"
        if (!lowerLimit.get()){
            status = "WARNING: Elevator is not at lowest position. Please turn the robot off and fully lower the elevator by hand."
        }
        SmartDashboard.putString("Status", status)
    }



    // Return true if it is unsafe to run the motor due to upper limit switch
    fun upperLimitHit(): Boolean {
        // if limit switch isn't triggered, it is safe to run motor
        if(upperLimit.get()) {
            return false
        }
        // if in voltage mode, don't run motor forwards
        if(targetVoltage != null) {
            return targetVoltage!! > 0.0
        }
        // if in pid mode, don't run towards a setpoint that is higher than current position
        else {
            return getPosition() < targetDist
        }
    }

    // Return true if it is unsafe to run the motor due to lower limit switch
    fun lowerLimitHit(): Boolean {
        // if limit switch isn't triggered, safe to run
        if(lowerLimit.get()) {
            return false
        }
        // if in voltage mode, don't run motor backwards
        if(targetVoltage != null) {
            return targetVoltage!! < 0.0
        }
        // if in pid mode, don't run towards a setpoint lower than current position
        else {
            return getPosition() > targetDist
        }
    }

    override fun periodic() {
        // if we reached the lower limit switch, reset encoder position to zero
        if (!lowerLimit.get()){
            resetEncoder()
        }

        // if the upper limit is hit and either
        if (upperLimitHit()){
            pid.setReference(0.0, kDutyCycle)
            pid.setIAccum(0.0)
        }
        else if (lowerLimitHit()){
            pid.setReference(0.0, kDutyCycle)
            pid.setIAccum(0.0)
            
        } else{
            if(targetVoltage != null) {
                pid.setReference(targetVoltage!!, kVoltage)
            } else {
                pid.setReference(targetDist, kPosition)
            }
        }

        if(debug) {
            SmartDashboard.putString("Winch Mode", if(targetVoltage != null) "Voltage" else "PID Position")
            SmartDashboard.putNumber("Winch Target Voltage", if(targetVoltage != null) targetVoltage!! else 0.0)
            SmartDashboard.putNumber("Winch Target Position", targetDist)
            SmartDashboard.putNumber("Winch Current Position", getPosition())
            SmartDashboard.putBoolean("Winch lower limit hit", !lowerLimit.get())
            SmartDashboard.putBoolean("Winch upper limit hit", !upperLimit.get())
        }
    }

    // set target direct speed for the motor
    fun setVoltage(voltage: Double?) {
        if (voltage != null && ((upperLimitHit() && voltage > 0.0) || (lowerLimitHit() && voltage < 0.0))) {
            // if at an upper limit and voltage we want to set is greater than 0, or the opposite at the bottom, set voltage to 0
            targetVoltage = 0.0
        } else {
            targetVoltage = voltage
        }
    }

    fun setTarget(distance: Double){
        if ((upperLimitHit() && distance > getPosition()) || (lowerLimitHit() && distance < getPosition())){
            // if at a lower / upper limit and distance exceeds the current encoder values, set target distance to the current position instead.
            targetDist = getPosition()
        } else{
            targetDist = distance
        }

    }


    fun getPosition(): Double {
        return encoder.getPosition()
    }

    fun resetEncoder() {
        encoder.setPosition(0.0)
    }

    fun atUpper() : Boolean{
        return upperLimit.get()
    }
    
    fun atLower() : Boolean{
        return lowerLimit.get()
    }
}