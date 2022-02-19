// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType

import frc.robot.subsystems.DrivetrainSubsystemOld

import frc.robot.commands.*

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    // MARK: Hardware initialization -- anything that needs a port
    val controller0 = XboxController(0)
    val controller1 = XboxController(1)

    val motorFrontLeft = CANSparkMax(4, MotorType.kBrushed)
    val motorBackLeft = CANSparkMax(3, MotorType.kBrushed)
    val motorFrontRight = CANSparkMax(1, MotorType.kBrushed)
    val motorBackRight = CANSparkMax(2, MotorType.kBrushed)

    val debugMotor: TalonFX =
    val debugSubsystem = MotorTestSubsystem(debugMotor)
    val gyro = AHRS()

    // MARK: Subsystems
    val drivetrain = DrivetrainSubsystemOld(
        motorFrontLeft,
        motorBackLeft,
        motorFrontRight,
        motorBackRight,
        gyro,
        0, 1,
        2, 3,
        400.0)

    init {
        configureButtonBindings()
    }

    /**
     * Controller ([GenericHID], [XboxController]) mapping.
     */
    private fun configureButtonBindings() {
        /*JoystickButton(controller0, XboxController.Button.kA.value).whenPressed(
            ArcadePIDDrive(drivetrain, controller0)
        )

        JoystickButton(controller0, XboxController.Button.kB.value).whenPressed(
            ArcadeDriveCommand(drivetrain, controller0)
        )*/

        drivetrain.defaultCommand = DirectDebugDrive(drivetrain, controller0)
        //drivetrain.defaultCommand = JoystickDrive(drivetrain, controller0)
        //debugSubsystem.defaultCommand = MotorTest(debugSubsystem, controller0)
    }


    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    //val autonomousCommand: Command
    //    get() =// An ExampleCommand will run in autonomous
    //        m_autoCommand
}