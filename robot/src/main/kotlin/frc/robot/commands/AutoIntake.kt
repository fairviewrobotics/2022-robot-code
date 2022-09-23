package frc.robot.commands

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.IntakeSubsystem
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.json.Json


class Ball(val color: String, val yaw: Double, val pitch: Double, val distance: Double, val score: Double)

class BallArray(val balls: Array<Ball>)
class AutoIntake(val system: IntakeSubsystem) : CommandBase() {
    init {
        addRequirements(system)
        system.defaultCommand = this

    }

    override fun execute() {
        val team = "Blue" //TODO: Idk how to get our team color, someone fix this
        val instance = NetworkTableInstance.getDefault()
        val table = instance.getTable("ML")
        val entry = table.getEntry("ballvision").toString()
        val ballvisionJSON = Json.decodeFromString<BallArray>(entry)

        var bestBall = ballvisionJSON.balls.get(0)
        for (ball in ballvisionJSON.balls) {
            if (ball.score > bestBall.score) {
                bestBall = ball
            }

        }

        if (team.lowercase() == bestBall.color.lowercase() && bestBall.score > 0.3 && bestBall.distance < 1) {
            println("No way this works")
            system.solenoids.set(DoubleSolenoid.Value.kForward)
            system.motor.setVoltage(12.0)
        } else {
            system.solenoids.set(DoubleSolenoid.Value.kReverse)
        }
    }


}