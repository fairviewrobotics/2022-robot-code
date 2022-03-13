package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import frc.robot.Constants
import frc.robot.subsystems.BallMotorSubsystem
import frc.robot.subsystems.ShooterSubsystem
import frc.robot.subsystems.WinchSubsystem
import java.lang.Math.abs

/*
Control the indexer and gate while shooting is happening.
This command does not control the ShooterSubsystem, but
monitors its speed.
 */
class ShootBallMotor(
    val shooter1: ShooterSubsystem,
    val shooter2: ShooterSubsystem,
    val shoot1Speed: () -> Double,
    val shoot2Speed: () -> Double,
    val gate: BallMotorSubsystem,
    val indexer: BallMotorSubsystem
    ) : CommandBase() {

    companion object {
        // tolerance (in rad/s) on shooter being at speed
        val speedTolerance = 7.0
    }

    // number of cycles that we have been at speed for
    var atSpeedCycles = 0
    // if we have goten to speed
    var atSpeedOnce = false

    // number of cycles that we have not been at speed for
    var belowSpeedCycles = 0
    // if we have gotten to speed and then slowed down,
    // ie, had a ball go through the shooter
    var belowSpeedOnce = false

    // if we got up to speed and then dropped (ie

    init {
        addRequirements(gate)
        addRequirements(indexer)
        // we don't add requirements on shooter1 and shooter2 because we are monitoring their speeds, not controlling them
    }

    // check if motors are at their target speed
    fun atSpeed(): Boolean {
        return abs(shooter1.getVelocity() - shoot1Speed()) <= speedTolerance &&
                abs(shooter2.getVelocity() - shoot2Speed()) <= speedTolerance
    }

    override fun execute() {
        val ready = atSpeed()
        // check if we are at speed
        if(ready) {
            atSpeedCycles++
            if(atSpeedCycles > 10) {
                atSpeedOnce = true
            }
            belowSpeedCycles = 0
        } else {
            atSpeedCycles = 0
            if(atSpeedOnce) {
                belowSpeedCycles++
                if(belowSpeedCycles > 10) {
                    belowSpeedOnce = true
                }
            }
        }

        if(ready) {
            // if we can shoot, run the gate
            gate.setSpeed(Constants.gateSpeed)
            // only run the indexer if we've already shot one ball
            // otherwise, running the indexer + gate will advance the second ball to far
            if(belowSpeedOnce) {
                indexer.setSpeed(Constants.indexerSpeed)
            } else {
                indexer.setSpeed(0.0)
            }
        } else {
            indexer.setSpeed(Constants.indexerSpeed)
            gate.setSpeed(0.0)
        }
    }

    override fun end(interrupted: Boolean) {
        gate.setSpeed(0.0)
        indexer.setSpeed(0.0)

        atSpeedOnce = false
        atSpeedCycles = 0
        belowSpeedOnce = false
        belowSpeedCycles = 0
    }

    override fun isFinished() = false
}

// Run the shooter and indexer/gate. This shoots any balls in the robot
fun ShootCommand(shooter1: ShooterSubsystem,
                 shooter2: ShooterSubsystem,
                 gate: BallMotorSubsystem,
                 indexer: BallMotorSubsystem,
                 shoot1Speed: () -> Double,
                 shoot2Speed: () -> Double,
                 ): Command {
    return ParallelCommandGroup(
        ShooterPID(shooter1, shoot1Speed),
        ShooterPID(shooter2, shoot2Speed),
        // run gate + magazine if shooters are running fast enough
        ShootBallMotor(shooter1, shooter2, shoot1Speed, shoot2Speed, gate, indexer)
    )
}