// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController.Button.*
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.DoubleSolenoid.Value
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

import com.revrobotics.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard


import frc.robot.subsystems.*


import frc.robot.commands.*

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    // Hardware and subsystem initialization

    val controller0 = XboxController(0)
    val controller1 = XboxController(1)

    // drivetrain
    val motorFrontLeft = CANSparkMax(Constants.driveFrontLeftID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val motorBackLeft = CANSparkMax(Constants.driveBackLeftID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val motorFrontRight = CANSparkMax(Constants.driveFrontRightID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val motorBackRight = CANSparkMax(Constants.driveBackRightID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val drivetrain = CANSparkMaxDrivetrainSubsystem(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, AHRS())

    // climber
    //val winchMotor = CANSparkMax(Constants.climbWinchID, CANSparkMaxLowLevel.MotorType.kBrushless)
    // TODO: attach limit switches directly to pins on Spark
    //val winch = WinchSubsystem(winchMotor, DigitalInput(0), DigitalInput(1))

    val climbSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.climbSolenoidLeftID.first,Constants.climbSolenoidLeftID.second) 
    val climbPneumatics = GenericSolenoidSubsystem(climbSolenoid)

    // shooter
    val shooterMotor1 = WPI_TalonFX(Constants.shooterLowID)
    val shooterMotor2  = WPI_TalonFX(Constants.shooterHighID)
    val shooter1 = TalonFXShooterSubsystem(shooterMotor1, 1.0)
    val shooter2 = TalonFXShooterSubsystem(shooterMotor2, -1.0)

    // intake / indexer / gate
    val intake = BallMotorSubsystem(WPI_TalonSRX(Constants.intakeID))
    val indexer = BallMotorSubsystem(WPI_TalonSRX(Constants.indexerID))
    val gate = BallMotorSubsystem(WPI_TalonSRX(Constants.gateID))
    
    // simultaneous pneumatics push and pull
    val climberPull = ParallelCommandGroup(
        GenericPneumaticCommand(climbPneumatics, DoubleSolenoid.Value.kReverse).withTimeout(1.0)
    )

    val climberPush = ParallelCommandGroup(
        GenericPneumaticCommand(climbPneumatics, DoubleSolenoid.Value.kForward).withTimeout(1.0)
    )


    // initial climb: drive up, lower climber fully, pull in pneumatics
    val initialClimb = SequentialCommandGroup(
        //LimitedWinchCommand(winch, { -1.0 }),
        //climberPull
    )

    // secondary climb: raise climber to half
    /*
    // ! all times are in seconds
    val secondaryClimb = SequentialCommandGroup(
        //FixedWinchSpeed(winch, { 1.0 }).withTimeout(0.5), // todo: tune
        climberPull,
        //LimitedWinchCommand(winch, { 1.0 }),
        climberPush,
        //FixedWinchSpeed(winch, { -1.0 }).withTimeout(0.5),
        //LimitedWinchCommand(winch, { -1.0 })
    )
    */

    init {
        configureButtonBindings()
    }

    /**
     * Controller ([GenericHID], [XboxController]) mapping.
     */
    private fun configureButtonBindings() {
        // run shooter when bumpers are held
        val shoot1Speed = { Constants.shooterRadPerS + Constants.shooterAdjustRadPerS }
        val shoot2Speed = { Constants.shooterRadPerS }
        JoystickButton(controller0, kLeftBumper.value).whenHeld(
            ShootCommand(shooter1, shooter2, gate, indexer, shoot1Speed, shoot2Speed)
        )

        JoystickButton(controller0, kX.value).whenHeld(
            ParallelCommandGroup(
                FixedShooterSpeed(shooter1, { 1.0 }),
                FixedShooterSpeed(shooter2, { 1.0 })
            )
        )

        // run intake on A
        JoystickButton(controller0, kA.value).whenHeld(
            FixedBallMotorSpeed(intake, { Constants.intakeSpeed })
        )

        // run intake + indexer on B
        JoystickButton(controller0, kB.value).whenHeld(
            ParallelCommandGroup(
                FixedBallMotorSpeed(intake, { Constants.intakeSpeed }),
                FixedBallMotorSpeed(indexer, { Constants.indexerSpeed })
            )
        )

        // run gate on Y
        JoystickButton(controller0, kY.value).whenHeld(
            FixedBallMotorSpeed(gate, { Constants.gateSpeed })
        )

        drivetrain.defaultCommand = DirectJoystickDrive(drivetrain, controller0)
        //drivetrain.defaultCommand = JoystickDrive(drivetrain, controller0)
        //debugSubsystem.defaultCommand = MotorTest(debugSubsystem, controller0)


        // run gate on secondary Y
        JoystickButton(controller1, kY.value).whenHeld(
            FixedBallMotorSpeed(gate, { Constants.gateSpeed })
        )

        // run magazine on secondary B
        JoystickButton(controller1, kB.value).whenHeld(
            FixedBallMotorSpeed(indexer, { Constants.indexerSpeed })
        )
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