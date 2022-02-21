/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ShooterSubsystem
import frc.robot.subsystems.WhichMotor
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
        shooterSubsystem.setCoast() // THIS MUST BE DONE, OTHERWISE MOTOR BREAKS
    }

    override fun execute() {
        // use 0.9 * feed forward to not go over speed
        val lowerSpeed = controller.calculate(-1.0 * shooterSubsystem.getVelocity(WhichMotor.LOWER), -1.0 * setPt()) - 0.9 * feedForward.calculate(setPt())
        val upperSpeed = controller.calculate(shooterSubsystem.getVelocity(WhichMotor.UPPER), setPt()) + 0.9 * feedForward.calculate(setPt())
        shooterSubsystem.setVoltage(lowerSpeed, WhichMotor.LOWER)
        shooterSubsystem.setVoltage(upperSpeed, WhichMotor.UPPER)
    }

    override fun end(interrupted: Boolean) {
        shooterSubsystem.setVoltage(0.0, WhichMotor.LOWER)
        shooterSubsystem.setVoltage(0.0, WhichMotor.UPPER)
    }

    override fun isFinished() = false
}
