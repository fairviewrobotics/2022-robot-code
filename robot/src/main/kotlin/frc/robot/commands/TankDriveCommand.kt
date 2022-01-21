package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.DrivetrainSubsystem
import kotlin.math.pow

object TankDriveCommandConstants {
    const val kLeftP = 0.1
    const val kLeftI = 0.1
    const val kLeftD = 0.1

    const val kRightP = 0.1
    const val kRightI = 0.1
    const val kRightD = 0.1
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
        // get vals
        val desiredLeftSpeed = (controller.leftY + controller.leftX).pow(5);
        val desiredRightSpeed = (-controller.leftY - controller.leftX).pow(5);
        val currentRightSpeed = drivetrain.wheelSpeeds.rightMetersPerSecond
        val currentLeftSpeed = drivetrain.wheelSpeeds.leftMetersPerSecond


        // calculate
        val leftSpeedToSet = leftPIDController.calculate(currentLeftSpeed, desiredLeftSpeed)
        val rightSpeedToSet = rightPIDController.calculate(currentRightSpeed, desiredRightSpeed)

        // drive
        drivetrain.tankDriveSpeed(leftSpeedToSet, rightSpeedToSet)

        if (Constants.debugMode) {
            println(
                """
                    TankDriveCommand Debug Dump:
                    -------------------------------------
                    desiredLeftSpeed:  $desiredLeftSpeed
                    desiredRightSpeed: $desiredRightSpeed
                    leftSpeedToSet:    $leftSpeedToSet
                    rightSpeedToSet:   $rightSpeedToSet
                    currentLeftSpeed:  $currentLeftSpeed
                    currentRightSpeed: $currentRightSpeed
                    -------------------------------------
                """
            )
        }
    }

    override fun isFinished() = false
}