// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.Compressor

class CompressorSubsystem(val compressor: Compressor) : SubsystemBase() {

    fun enable(){
        compressor.enableDigital()
    }

    fun disable(){
        compressor.disable()
    }
    
    fun switchValue() : Boolean {
        return compressor.getPressureSwitchValue() // true if pressure low, false otherwise
    }

    override fun periodic() {
        // This method will be called once per scheduler run
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}