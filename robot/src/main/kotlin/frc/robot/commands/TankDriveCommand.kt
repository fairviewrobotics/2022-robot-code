package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.DrivetrainSubsystem

object TankDriveCommandConstants {
    const val kLeftP = 1.0
    const val kLeftI = 1.0
    const val kLeftD = 1.0

    const val kRightP = 1.0
    const val kRightI = 1.0
    const val kRightD = 1.0
}

class TankDriveCommand(val drivetrain: DrivetrainSubsystem, val controller: XboxController) : CommandBase() {
    val leftPIDController = PIDController(
        TankDriveCommandConstants.kLeftP,
        TankDriveCommandConstants.kLeftI,
        TankDriveCommandConstants.kLeftD
    )

    val rightPIDController = PIDController(
        TankDriveCommandConstants.kRightP,
        TankDriveCommandConstants.kRightI,
        TankDriveCommandConstants.kRightD
    )

    init {
        addRequirements(drivetrain)
    }

    override fun execute() {
        // get controller vals

        // calculate

        // drive
    }

    override fun isFinished() = false
}