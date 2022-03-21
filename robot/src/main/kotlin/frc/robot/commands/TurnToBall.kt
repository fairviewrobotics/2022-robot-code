package frc.robot.commands

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.DrivetrainSubsystem

object BallVisionNT {
    val ntInst = NetworkTableInstance.getDefault()
    val table = ntInst.getTable("ML")
    val angle = table.getEntry("targetAngle")
    val foundTarget = table.getEntry("foundTarget")
}


class TurnToBall(val controller: XboxController, val drivetrain: DrivetrainSubsystem) : CommandBase() {
    val control = TurnToAngleController(drivetrain)

    override fun execute() {
        val targetAngle = {
            if (BallVisionNT.foundTarget.getBoolean(false))
                drivetrain.heading + BallVisionNT.angle.getDouble(0.0) / 2.0
            else
                drivetrain.heading
        }
        control.execute(
            targetAngle,
            4.0 * (controller.leftY + controller.rightY * Constants.kDrivetrainFineForwardSpeed)
        )
    }

    override fun isFinished(): Boolean {
        //return !BallVisionNT.foundTarget.getBoolean(false)
        return false
    }
}