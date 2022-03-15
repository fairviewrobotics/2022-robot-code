package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.DoubleSolenoid

class SolenoidSubsystem(val output: DoubleSolenoid) : SubsystemBase() {
    fun set(state: DoubleSolenoid.Value) {
        output.set(state)
    }
}