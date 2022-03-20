package frc.robot.commands

import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj.DoubleSolenoid.Value
import com.revrobotics.SparkMaxPIDController.AccelStrategy
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.subsystems.WinchSubsystem
import frc.robot.Constants
import frc.robot.subsystems.SolenoidSubsystem
import kotlin.math.abs

// Run the winch at a set voltage
class FixedWinchVoltage(val climber: WinchSubsystem, val voltage: () -> Double) : CommandBase() {
    init {
        addRequirements(climber)
        climber.pid.setOutputRange(-Constants.elevatorMaxVoltage, Constants.elevatorMaxVoltage)
        climber.winch.setSmartCurrentLimit(Constants.elevatorMaxCurrent.toInt())
        climber.winch.setSecondaryCurrentLimit(Constants.elevatorMaxCurrent)
        climber.winch.setIdleMode(CANSparkMax.IdleMode.kBrake)
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
        climber.winch.setSmartCurrentLimit(Constants.elevatorMaxCurrent.toInt())
        climber.winch.setSecondaryCurrentLimit(Constants.elevatorMaxCurrent)
        climber.winch.setIdleMode(CANSparkMax.IdleMode.kBrake)
        addRequirements(climber)
    }

    override fun initialize() {
        climber.pid.setP(Constants.elevatorP)
        climber.pid.setI(Constants.elevatorI)
        climber.pid.setD(Constants.elevatorD)
        climber.pid.setOutputRange(-Constants.elevatorMaxVoltage, Constants.elevatorMaxVoltage)
        climber.pid.setSmartMotionMaxAccel(Constants.elevatorMaxAccel,0) // in rpm per sec
        climber.pid.setSmartMotionMaxVelocity(Constants.elevatorMaxVel, 0) // in rpm

        //climber.pid.setIZone(Constants.elevatorIZ) // - to + range where i parameter should take effect
        //climber.pid.setFF(Constants.elevatorFF)

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
        // first step: getting onto bar
        WinchPIDCommand(climber) { Constants.elevatorMaxPos }, // raise elevator
        WinchPIDCommand(climber) { Constants.elevatorMinPos }, // lower elevator
        // second step: pneumatics handoff
        PneumaticCommand(solenoid, Value.kReverse).withTimeout(1.0), // deploy pneumatics in reverse, locking onto bar
        WinchPIDCommand(climber) {Constants.elevatorMaxPos * 0.5}, //half-raise climber to release
        PneumaticCommand(solenoid, Value.kReverse).withTimeout(1.0), // push the robot backwards
        // third step: getting onto second bar
        // todo: not so sure about this one
        WinchPIDCommand(climber) { Constants.elevatorMaxPos }, // raise elevator
        PneumaticCommand(solenoid, Value.kForward).withTimeout(1.0), // release pneumatics
        WinchPIDCommand(climber) {Constants.elevatorMinPos }, // lower elevator
        PneumaticCommand(solenoid, Value.kForward).withTimeout(1.0), // release pneumatics





    )
}