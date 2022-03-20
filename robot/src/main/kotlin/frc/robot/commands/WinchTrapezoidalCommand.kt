package frc.robot.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.robot.Constants
import frc.robot.subsystems.WinchSubsystem
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand

/** A command that will turn the robot to the specified angle using a motion profile.  */
class WinchTrapezoidal(targetDistance: Double, winch: WinchSubsystem) : ProfiledPIDCommand(
        ProfiledPIDController(
                Constants.elevatorP,
                Constants.elevatorI,
                Constants.elevatorD,
                TrapezoidProfile.Constraints(
                        Constants.elevatorMaxVel,
                        Constants.elevatorMaxAccel)),  // Close loop on heading
        winch::getPosition,  // Set reference to target
        targetDistance,  // Pipe output to turn robot
        { output: Double, _ -> winch.setVoltage(output) },  // Require the drive
        winch) {
    val winch = winch
    init {
        //getController().enableContinuousInput(-180, 180)
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController()
                .setTolerance(Constants.elevatorCLErr, 0.0001)
    }
    // End when the controller is at the reference.
    override fun isFinished(): Boolean{
        return getController().atGoal() || (winch.atLower() || winch.atUpper())
    }
}