// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.SparkMaxPIDController
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import frc.robot.Constants
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

class WinchSubsystem(val winch: CANSparkMax,
                     val lowerLimit: DigitalInput, 
                     val upperLimit: DigitalInput) : SubsystemBase() {
    var actualSpeed = 0.0
    var targetSpeed = 0.0
    val pid = winch.getPIDController()
    var encoder = winch.getEncoder()

    override fun periodic() {
        /*
        // 50% increase / 1 second
         
        if ((actualSpeed - targetSpeed) > (.5 / 50)) {
            val update = sign(targetSpeed) * (.5 / 50)
            actualSpeed += update
        } else {
            targetSpeed = actualSpeed
        }
        */
        // todo: experiment with software limits: if encoder hits a value, stop
        if (lowerLimit.get()) {
            resetEncoder()
            
            winch.setVoltage(max(0.0, actualSpeed)) // todo: check correct return if equal
        }

        if (upperLimit.get()) {
            encoder.setPosition(Constants.climbMaxVal)
            winch.setVoltage(min(0.0, actualSpeed))
        }
    }

    // set target voltage for the motor
    // the voltage will be lowered to this voltage at the acceleration limit
    fun setVoltage(voltage: Double) {
        targetSpeed = voltage
    }

    fun getPosition(): Double {
        return encoder.getPosition()
    }

    fun resetEncoder() {
        encoder.setPosition(0.0)
    }

    //var distance = encoder.position * 16 * (2 * 3.14159 * 3.175)
    var hitUpper = upperLimit.get()
    var hitLower = lowerLimit.get()
}