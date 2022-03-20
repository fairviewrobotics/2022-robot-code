// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMax.ControlType.*
import com.revrobotics.SparkMaxPIDController.AccelStrategy
import com.revrobotics.SparkMaxPIDController
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

class WinchSubsystem(val winch: CANSparkMax,
                     val lowerLimit: DigitalInput, 
                     val upperLimit: DigitalInput) : SubsystemBase() {
    var actualSpeed = 0.0
    val motor = winch
    var targetSpeed = 0.0
    var targetDist = 0.0
    val pid = winch.getPIDController()
    var encoder = winch.getEncoder()
    var isPID = true

    override fun periodic() {
        /* 
        if (isPID){
            
            pid.setIZone(Constants.elevatorIZ)
            pid.setFF(Constants.elevatorFF)
            pid.setOutputRange(Constants.elevatorMin, Constants.elevatorMax)
            pid.setSmartMotionMaxAccel(Constants.elevatorMaxAccel,0)
            pid.setSmartMotionMaxVelocity(Constants.elevatorMaxVel, 0)
            pid.setSmartMotionMinOutputVelocity(Constants.elevatorMinVel, 0)
            pid.setSmartMotionAllowedClosedLoopError(Constants.elevatorCLErr, 0)
            pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0)
        }
        */
        

        if (!isPID){
            // 50% increase / 1 second
            
            if ((actualSpeed - targetSpeed) > (.5 / 50)) {
                val update = sign(targetSpeed) * (.5 / 50)
                actualSpeed += update
            } else {
                targetSpeed = actualSpeed
            }
            
            if (!upperLimit.get() || !lowerLimit.get()){
                winch.setVoltage(0.0)
                targetSpeed = 0.0
            } else{
                winch.setVoltage(targetSpeed)
            }
        }

        
        // todo: experiment with software limits: if encoder hits a value, stop
        if (!lowerLimit.get()){
            resetEncoder()
        }
        
        if (!upperLimit.get() && getPosition() < targetDist){ // todo: set
            pid.setReference(0.0, kDutyCycle)
            pid.setIAccum(0.0)
        }
        else if (!lowerLimit.get() && getPosition() > targetDist){
            pid.setReference(0.0, kDutyCycle)
            pid.setIAccum(0.0)
            targetDist = 0.0
            
        } else{
            pid.setReference(targetDist, kPosition)
        }
        SmartDashboard.putNumber("target", targetDist)
            
    }

    // set target voltage for the motor
    // the voltage will be lowered to this voltage at the acceleration limit
    fun setVoltage(voltage: Double) {
        targetSpeed = voltage
    }

    fun setTarget(distance: Double){
        targetDist = distance
    }


    fun getPosition(): Double {
        return encoder.getPosition()
    }

    fun resetEncoder() {
        encoder.setPosition(0.0)
    }

    //var distance = encoder.position * 16 * (2 * 3.14159 * 3.175)
    fun atUpper() : Boolean{
        return upperLimit.get()
    }
    
    fun atLower() : Boolean{
        return lowerLimit.get()
    }
}