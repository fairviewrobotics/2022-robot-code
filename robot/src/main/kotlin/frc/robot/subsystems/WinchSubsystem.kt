// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.motorcontrol.MotorController

class WinchSubsystem(val winch: MotorController, 
                     val lowerLimit: DigitalInput, 
                     val upperLimit: DigitalInput) : SubsystemBase() {
    // output speed to set motor at
    var output = 0.0

    fun setSpeed(speed: Double){
        output = speed
    }

    fun atUpper() : Boolean{
        return upperLimit.get()
    }

    fun atLower() : Boolean{
        return lowerLimit.get()
    }

    override fun periodic() {
        if (output > 0){
            if (!upperLimit.get()){
                winch.set(output)
            } else{
                winch.set(0.0)
            }
        } else {
            if (!lowerLimit.get()){
                winch.set(output)
            } else{
                winch.set(0.0)
            }
        }
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}