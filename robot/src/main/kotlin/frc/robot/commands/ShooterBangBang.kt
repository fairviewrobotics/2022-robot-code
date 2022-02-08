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
 * Drive the shooter based on some setpoint (probably not a direct mapping to a joystick value), using a Bang-Bang control scheme.
 */
class ShooterBangBang(val shooterSubsystem: ShooterSubsystem, val setPt: () -> Double) : CommandBase() {
    val controller = BangBangController()
    val enc = shooterSubsystem.getEncoder()
    val feedForwardLower = SimpleMotorFeedforward(Constants.shooterFFS, Constants.shooterFFV, Constants.shooterFFA)
    init {
        addRequirements(shooterSubsystem)
        shooterSubsystem.setCoast() // THIS MUST BE DONE, OTHERWISE MOTOR BREAKS
    }

    override fun execute() {
        if (shooterSubsystem.isCoast()) {
            // this verification is probably unnecessary but
            shooterSubsystem.setSpeed(controller.calculate(enc.getVelocity(), setPt()) + 0.8 * feedForward.calculate(setPt()))
            // the tutorial used 0.9 as the modifier for feed forward, but recommended not setting it too high so as to not break things
        }
    }

    override fun end(interrupted: Boolean) {
        shooterSubsystem.setSpeed(0.0)
    }

    override fun isFinished() = false
}
