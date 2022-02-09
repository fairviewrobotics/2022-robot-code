// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.motorcontrol.MotorController

class ClimberSubsystem(val solenoid: DoubleSolenoid, 
                       val winch: MotorController, 
                       val lowerLimit: DigitalInput, 
                       val upperLimit: DigitalInput) : SubsystemBase() {

    fun setInitial(speed: Double){
        if (speed > 0){
            if (!upperLimit.get()){
                winch.set(speed)
            } else{
                winch.set(0.0)
            }
        } else {
            if (!lowerLimit.get()){
                winch.set(speed)
            } else{
                winch.set(0.0)
            }
        }
        
    }

    fun set(state: DoubleSolenoid.Value) {
        solenoid.set(state)
    }

    override fun periodic() {
        // This method will be called once per scheduler run
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}