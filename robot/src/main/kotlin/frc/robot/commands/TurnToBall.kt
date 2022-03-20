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
}


class TurnToBall(val controller: XboxController, val drivetrain: DrivetrainSubsystem) : CommandBase() {
    val control = TurnToAngleController(drivetrain)

    override fun execute() {
        control.execute(
            { drivetrain.heading + BallVisionNT.angle.getDouble(0.0) / 1.2 },
            4.0 * (controller.leftY + controller.rightY * Constants.kDrivetrainFineForwardSpeed)
        )
    }
}