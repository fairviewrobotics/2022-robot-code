package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj.DoubleSolenoid.Value
import com.revrobotics.SparkMaxPIDController.AccelStrategy
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
        if (abs(voltage()) <= Constants.elevatorMaxVoltage){
            climber.setVoltage(voltage())
        } else{
            climber.setVoltage(0.0)
        }

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
        climber.pid.setOutputRange(-1.0*Constants.elevatorMaxVoltage, Constants.elevatorMaxVoltage) // in power (likely v?)
        climber.pid.setSmartMotionMaxAccel(Constants.elevatorMaxAccel,0) // in rpm per sec
        climber.pid.setSmartMotionMaxVelocity(Constants.elevatorMaxVel, 0) // in rpm

        climber.pid.setIZone(Constants.elevatorIZ) // - to + range where i parametre should take effect
        climber.pid.setFF(Constants.elevatorFF)

        climber.pid.setSmartMotionAllowedClosedLoopError(Constants.elevatorPosTolerance, 0)
        climber.pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0)

        climber.encoder.setPositionConversionFactor(1.0)

        climber.setVoltage(null)
    }

    override fun execute() {
        if (setPt() <= Constants.elevatorMaxPos && setPt() >= Constants.elevatorMinPos){
            climber.setTarget(setPt())
        } else{
            climber.setVoltage(0.0)
        }

    }

    override fun isFinished(): Boolean {
        return abs(climber.getPosition() - setPt()) <= Constants.elevatorPosTolerance // should be redundant in smart motion mode, but eh.

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
        // first step: getting onto bar
        WinchPIDCommand(climber) { Constants.elevatorTopPosition }, // raise elevator todo: should this be same value as elevatorMaxPos?
        WinchPIDCommand(climber) { 0.0 }, // lower elevator
        // second step: pneumatics handoff
        PneumaticCommand(solenoid, Value.kReverse), // deploy pneumatics in reverse, locking onto bar
        WinchPIDCommand(climber) {Constants.elevatorTopPosition * 0.5}, //half-raise climber to release
        PneumaticCommand(solenoid, Value.kReverse), // push the robot backwards
        // third step: getting onto second bar
        // todo: not so sure about this one
        WinchPIDCommand(climber) { Constants.elevatorTopPosition }, // raise elevator todo: should this be same value as elevatorMaxPos?
        PneumaticCommand(solenoid, Value.kForward), // release pneumatics
        WinchPIDCommand(climber) { 0.0 }, // lower elevator
        PneumaticCommand(solenoid, Value.kForward), // release pneumatics





    )
}