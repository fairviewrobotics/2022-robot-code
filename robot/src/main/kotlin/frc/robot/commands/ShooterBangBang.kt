/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ShooterSubsystem
import edu.wpi.first.math.controller.BangBangController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import frc.robot.Constants

/**
 * Drive the shooter at some angular velocity setpoint (radians / s), using a Bang-Bang control scheme.
 */
class ShooterBangBang(val shooterSubsystem: ShooterSubsystem, val setPt: () -> Double) : CommandBase() {
    val controller = BangBangController()
    val feedForward = SimpleMotorFeedforward(Constants.shooterFFS, Constants.shooterFFV, Constants.shooterFFA)
    init {
        addRequirements(shooterSubsystem)
        shooterSubsystem.setCoast()
    }

    override fun execute() {
        // use 0.9 * feed forward to not go over speed
        val speed = controller.calculate(shooterSubsystem.getVelocity(), setPt()) + 0.9 * feedForward.calculate(setPt())
        shooterSubsystem.setSpeed(speed)
    }

    override fun end(interrupted: Boolean) {
        shooterSubsystem.setSpeed(0.0)
    }

    override fun isFinished() = false
}
