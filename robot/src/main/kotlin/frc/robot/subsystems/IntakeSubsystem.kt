package frc.robot.subsystems

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj2.command.SubsystemBase

class IntakeSubsystem(val solenoids: DoubleSolenoid, val motor: MotorController): SubsystemBase() {
    fun setMotorVoltage(volts: Double) {
        motor.setVoltage(volts)
    }
    fun setSolenoidsState(state: DoubleSolenoid.Value) {
        solenoids.set(state)
    }

    override fun periodic() {
        // This method will be called once per scheduler run
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}