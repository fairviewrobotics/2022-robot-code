package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.motorcontrol.MotorController

// A ball motor is one of the intake / indexer / gate that manipulates balls
class BallMotorSubsystem(val motor: MotorController) : SubsystemBase(){

    fun setSpeed(speed: Double){
        motor.set(speed)
    }


    override fun periodic() {
        // This method will be called once per scheduler run
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}