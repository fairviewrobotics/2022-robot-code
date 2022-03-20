package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.WinchSubsystem
import frc.robot.Constants
import frc.robot.subsystems.SolenoidSubsystem
import kotlin.math.abs

// Run the winch at a set voltage
class FixedWinchVoltage(val climber: WinchSubsystem, val voltage: () -> Double) : CommandBase() {
    init {
        addRequirements(climber)
    }

    override fun execute() {
        climber.setVoltage(voltage())
    }

    override fun end(interrupted: Boolean) {
        // set climber to hold it's current position
        climber.setTarget(climber.getPosition())
        climber.setVoltage(null)
    }
}

// Run the winch to a specific position using the spark max's pid controller
class WinchPIDCommand(val climber: WinchSubsystem, val setPt: () -> Double) : CommandBase() {
    init {
        addRequirements(climber)
    }

    override fun initialize() {
        climber.pid.setP(Constants.elevatorP)
        climber.pid.setI(Constants.elevatorI)
        climber.pid.setD(Constants.elevatorD)
        climber.pid.setOutputRange(-1.0, 1.0)
        climber.pid.setSmartMotionMaxVelocity(1.0, 0)
        climber.pid.setSmartMotionMaxAccel(1.0, 0)

        climber.encoder.setPositionConversionFactor(1.0)

        climber.setVoltage(null)
    }

    override fun execute() {
        climber.setTarget(setPt())
    }

    override fun isFinished(): Boolean {
        return abs(climber.getPosition() - setPt()) <= Constants.elevatorPosTolerance
    }

    override fun end(interrupted: Boolean) {
        // set winch to maintan position
        climber.setTarget(climber.getPosition())
    }
}

// Autoclimb sequence
fun AutoClimb(climber: WinchSubsystem, solenoid: SolenoidSubsystem): Command {
    // TODO: Get correct sequence + timing necessary to climb
    return SequentialCommandGroup(
        WinchPIDCommand(climber) { Constants.elevatorTopPosition },
        WinchPIDCommand(climber) { 0.0 }
    )
}