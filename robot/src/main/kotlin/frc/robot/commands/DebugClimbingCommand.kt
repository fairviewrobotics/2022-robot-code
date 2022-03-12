package frc.robot.commands

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.WinchSubsystem

class DebugClimbingCommand(val climber: WinchSubsystem, val controller: XboxController) : CommandBase() {
    init {
        addRequirements(climber)
    }

    var speed = 0.0;
    override fun execute() {
        if (controller.aButtonPressed) {
            speed += 0.1
        }

        if (controller.bButtonPressed) {
            speed -= 0.1
        }

        if (controller.xButtonPressed) {
            climber.resetEncoder()
        }

        climber.set(speed + (-controller.leftY))

        SmartDashboard.putNumber("Speed", speed)
        SmartDashboard.putNumber("Controller Y", -controller.leftY)
        SmartDashboard.putNumber("Target Speed", climber.targetSpeed)
        SmartDashboard.putNumber("Actual Speed", climber.actualSpeed)
        SmartDashboard.putNumber("Position", climber.distance)
        SmartDashboard.putBoolean("Lower Limit Hit", climber.hitLower)
        SmartDashboard.putBoolean("Upper Limit Hit", climber.hitUpper)
    }
}