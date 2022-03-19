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

    override fun initialize() {
        climber.pid.setP(Constants.elevatorP)
        climber.pid.setI(Constants.elevatorI)
        climber.pid.setD(Constants.elevatorD)
        climber.pid.setOutputRange(-1.0, 1.0)
        climber.pid.setSmartMotionMaxVelocity(1.0, 0)
        climber.pid.setSmartMotionMaxAccel(1.0, 0)

        climber.encoder.setPositionConversionFactor(1.0)
        climber.winch.setSmartCurrentLimit(40)
        climber.winch.setSecondaryCurrentLimit(40.0)
    }

    var targetDist = 1.0;
    val stepSize = 1
    override fun execute() {

        if (controller.getAButton()) {
            targetDist += stepSize
            climber.pid.setIAccum(0.0)
            
        }

        if (controller.getBButton()) {
            targetDist -= stepSize
            climber.pid.setIAccum(0.0)
        }

        if (controller.getXButton()){
            targetDist = 5.0
        }
        if (controller.getYButton()){
            targetDist = Constants.climbMaxVal
        }
        if (!climber.atLower()){
            targetDist = 0.0
        }

        /*
        if (controller.xButtonPressed) {
            climber.resetEncoder()
        }
        */
        
        //climber.setPosition(targetDist)
        climber.setTarget(targetDist)

        SmartDashboard.putNumber("Target Position", targetDist)
        SmartDashboard.putNumber("velocity", climber.encoder.getVelocity())
        SmartDashboard.putNumber("Actual Position", climber.getPosition())
        SmartDashboard.putBoolean("Lower Limit Hit", climber.atLower())
        SmartDashboard.putBoolean("Upper Limit Hit", climber.atUpper())
    }
}