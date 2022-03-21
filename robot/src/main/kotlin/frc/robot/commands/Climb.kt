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

// Climb sequence when the robot starts on a bar with pneumatics hooked
fun ClimbToNext(climber: WinchSubsystem, solenoid: SolenoidSubsystem): Command {
    return SequentialCommandGroup(
        // Raise climber
        WinchPIDCommand(climber) {Constants.elevatorMaxPos * 0.33},
        // Move pneumatics backward then forwards to tilt robot toward bar
        PneumaticCommand(solenoid, Value.kReverse).withTimeout(2.0),
        PneumaticCommand(solenoid, Value.kForward).withTimeout(5.0),
        // Raise climber to reach bar distance
        WinchPIDCommand(climber) { Constants.elevatorMaxPos },
        // Move pneumatics backwards to grab bar
        PneumaticCommand(solenoid, Value.kReverse).withTimeout(2.0),
        // Lower elevator to climb bar
        WinchPIDCommand(climber) {Constants.elevatorMinPos },
        // Move pneumatics forward to grab bar
        PneumaticCommand(solenoid, Value.kForward).withTimeout(2.0),
    )
}

// Automatic climbing sequence.
// This should be started with the climber raised over the bar.
fun AutoClimb(climber: WinchSubsystem, solenoid: SolenoidSubsystem): Command {
    return SequentialCommandGroup(
        // Lower climber to bottom, lifting the robot
        WinchPIDCommand(climber) { Constants.elevatorMinPos },
        // Move pneumatics forward to grab bar
        PneumaticCommand(solenoid, Value.kForward).withTimeout(2.0),

        ClimbToNext(climber, solenoid),
        ClimbToNext(climber, solenoid)
    )
}