package frc.robot.commands

import com.beust.klaxon.Klaxon
import com.fasterxml.jackson.databind.util.JSONPObject
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.BallVisonJSON
import frc.robot.subsystems.IntakeSubsystem

class AutoIntake(val system: IntakeSubsystem) : CommandBase() {
    init {
        addRequirements(system)
        system.defaultCommand = this
    }

    override fun execute() {
        val team = "Blue" //TODO: Idk how to get our team color, someone fix this
        val instance = NetworkTableInstance.getDefault()
        val table = instance.getTable("ML")
        val ballvisionJSON = Klaxon().parse<BallVisonJSON>(table.toString()) ?: return


        if (team.lowercase() == ballvisionJSON.color.lowercase() && ballvisionJSON.score > 0.3 && ballvisionJSON.distance < 2) {
            system.solenoids.set(DoubleSolenoid.Value.kForward)
        } else {
            system.solenoids.set(DoubleSolenoid.Value.kReverse)
        }
    }


}