// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

class WinchSubsystem(val winch: CANSparkMax,
                     val lowerLimit: DigitalInput, 
                     val upperLimit: DigitalInput) : SubsystemBase() {
    var actualSpeed = 0.0
    var targetSpeed = 0.0
    val encoder = winch.encoder

    override fun periodic() {
        // 50% increase / 1 second
        if ((actualSpeed - targetSpeed) > (.5 / 50)) {
            val update = sign(targetSpeed) * (.5 / 50)
            actualSpeed += update
        } else {
            targetSpeed = actualSpeed
        }

        if (lowerLimit.get()) {
            resetEncoder()
            winch.set(min(0.0, actualSpeed))
        }

        if (upperLimit.get()) {
            winch.set(max(0.0, actualSpeed))
        }
    }

    // Have fun clamping this!!!!
    fun set(percent: Double) {
        targetSpeed = percent
    }

    fun resetEncoder() {
        encoder.position = 0.0
    }

    var distance = encoder.position * 16 * (2 * 3.14159 * 3.175)
    var hitUpper = upperLimit.get()
    var hitLower = lowerLimit.get()
}