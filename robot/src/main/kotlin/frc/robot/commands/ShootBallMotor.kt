package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.BallMotorSubsystem
import frc.robot.subsystems.ShooterSubsystem
import frc.robot.subsystems.WinchSubsystem
import java.lang.Math.abs

/*
Set a ball motor to the speed supplied by the speed lambda.
This command does not halt its execution.
 */
class ShootBallMotor(
    val shooter1: ShooterSubsystem,
    val shooter2: ShooterSubsystem,
    val shoot1Speed: () -> Double,
    val shoot2Speed: () -> Double,
    val gate: BallMotorSubsystem,
    val indexer: BallMotorSubsystem
    ) : CommandBase() {

    // number of cycles that we have been at speed for
    var atSpeedCycles = 0
    // if we have gotten to speed
    var atSpeedOnce = false

    // if we got up to speed and then dropped (ie

    init {
        addRequirements(gate)
        addRequirements(indexer)
        // we don't add requirements on shooter1 and shooter2 because we are monitoring their speeds, not controlling them
    }

    // check if motors are at their target speed
    fun atSpeed(): Boolean {
        return abs(shooter1.getVelocity() - shoot1Speed()) < 7.0 &&
                abs(shooter2.getVelocity() - shoot2Speed()) < 7.0
    }

    override fun execute() {
        val ready = atSpeed()
        // check if we are at speed
        if(ready) {
            atSpeedCycles++
            if(atSpeedCycles > 5) {
                atSpeedOnce = true
            }
        } else {
            atSpeedCycles = 0
        }

        if(ready) {
            gate.setSpeed(Constants.gateSpeed)
            indexer.setSpeed(0.0)
        } else {
            if(atSpeedOnce) {
                indexer.setSpeed(Constants.indexerSpeed)
            } else {
                indexer.setSpeed(0.0)
            }

            gate.setSpeed(0.0)
        }
    }

    override fun end(interrupted: Boolean) {
        gate.setSpeed(0.0)
        indexer.setSpeed(0.0)

        atSpeedOnce = false
        atSpeedCycles = 0
    }

    override fun isFinished() = false
}