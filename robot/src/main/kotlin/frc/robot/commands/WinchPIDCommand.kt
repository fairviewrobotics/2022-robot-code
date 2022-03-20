package frc.robot.commands

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import com.revrobotics.CANSparkMax.ControlType.*
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.WinchSubsystem
import frc.robot.Constants

class WinchPIDCommand(val climber: WinchSubsystem, val controller: XboxController) : CommandBase() {
    init {
        addRequirements(climber)
    }

    var targetDist = 1.0;
    val stepSize = 1
    override fun execute() {
        if (controller.aButtonPressed) {
            if (targetDist <= Constants.climbMaxVal-stepSize){
                targetDist += stepSize
            }
            
        }

        if (controller.bButtonPressed) {
            if (targetDist >= stepSize){
                targetDist -= stepSize
            }
            
        }

        /*
        if (controller.xButtonPressed) {
            climber.resetEncoder()
        }
        */
        
        climber.pid.setReference(targetDist, kPosition)

        SmartDashboard.putNumber("Target Position", targetDist)
        SmartDashboard.putNumber("Actual Position", climber.getPosition())
        SmartDashboard.putBoolean("Lower Limit Hit", climber.hitLower)
        SmartDashboard.putBoolean("Upper Limit Hit", climber.hitUpper)
    }
}