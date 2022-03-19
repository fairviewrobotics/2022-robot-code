// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.XboxController.Button.*
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

import com.revrobotics.*
import edu.wpi.first.wpilibj2.command.button.POVButton
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger


import frc.robot.subsystems.*


import frc.robot.commands.*
import java.lang.Math.PI

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
    val climbPneumatics = SolenoidSubsystem(climbSolenoid)

    // shooter
    val shooterMotor1 = WPI_TalonFX(Constants.shooterLowID)
    val shooterMotor2  = WPI_TalonFX(Constants.shooterHighID)
    val shooter1 = TalonFXShooterSubsystem(shooterMotor1, 1.0)
    val shooter2 = TalonFXShooterSubsystem(shooterMotor2, -1.0)

    // intake / indexer / gate
    val intake = BallMotorSubsystem(WPI_TalonSRX(Constants.intakeID))
    val indexer = BallMotorSubsystem(WPI_TalonSRX(Constants.indexerID))
    val gate = BallMotorSubsystem(WPI_TalonSRX(Constants.gateID))

    // gate color sensor
    val colorSensor = ColorSensorV3(I2C.Port.kOnboard)

    // simultaneous pneumatics push and pull
    val climberPull = ParallelCommandGroup(
        PneumaticCommand(climbPneumatics, DoubleSolenoid.Value.kReverse).withTimeout(1.0)
    )

    val climberPush = ParallelCommandGroup(
        PneumaticCommand(climbPneumatics, DoubleSolenoid.Value.kForward).withTimeout(1.0)
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
        // use d-pad for turn to angle
        for (i in 0 until 8) {
            val angleDeg = 45 * i
            POVButton(controller0, angleDeg).whenHeld(
                TurnToAngle(drivetrain, { angleDeg * PI / 180.0 }, 0.0)
            )
        }

        // run shooter + vision on controller0 right bumper
        // See https://blackknightsrobotics.slack.com/files/UML602T96/F0377HEMXU3/image_from_ios.jpg For the control scheme.

        // PRIMARY DRIVER

        // LT - Vision Lineup
        Trigger { controller0.leftTriggerAxis > 0.2 }.whileActiveOnce(
            SequentialCommandGroup(
                CheckVisionOrRumble(controller0),
                TurnToHighGoal(drivetrain)
            )
        )

        // LB - Fine Drive, Left Joystick - Normal Drive, Right Joystick - Inverted Drive
        drivetrain.defaultCommand = DualStickArcadeDrive(drivetrain, controller0)

        // RT - Set Manual Shooting Power
        Trigger { controller0.rightTriggerAxis > 0.2 }.whenActive(
            ParallelCommandGroup(
                FixedShooterSpeed(shooter1, { controller0.rightTriggerAxis }),
                FixedShooterSpeed(shooter2, { controller0.rightTriggerAxis })
            )
        )

        // RB - Visual Shooting
        JoystickButton(controller0, kRightBumper.value).whenHeld(
            ShootVision(drivetrain, shooter1, shooter2, gate, indexer, controller0)
        )

        // X - Gate Forward
        JoystickButton(controller0, kX.value).whenHeld(
            FixedBallMotorSpeed(gate, { Constants.gateSpeed })
        )

        // B - Run Intake
        JoystickButton(controller0, kB.value).whenHeld(
            FixedBallMotorSpeed(intake, { Constants.intakeSpeed })
        )

        // Y - Pneumatic Intake Up TODO
        // A - Pneumatic Intake Down TODO


        // SECONDARY DRIVER

        // LT - Climber Down TODO
        // RT - Climber Up TODO


        // LB - Auto Climb TODO

        // RB - Run Intake/Indexer/Gate
        Trigger { controller1.rightTriggerAxis > 0.2 }.whileActiveOnce(
            ParallelCommandGroup(
                FixedBallMotorSpeed(intake, { Constants.intakeSpeed }),
                FixedBallMotorSpeed(indexer, { Constants.indexerSpeed }),
                GateSensored(gate, { Constants.gateSpeed }, colorSensor)
            )
        )

        // Y - Direct shooter
        JoystickButton(controller1, kY.value).whileHeld(
            ParallelCommandGroup(
                DualShooterPID(shooter1, shooter2) { DualShootSpeed(Constants.shooterRadPerS, Constants.shooterAdjustRadPerS) },
                ShootBallMotor(shooter1, shooter2, gate, indexer),
                MaintainAngle(drivetrain)
            )
        )

        // B - Reverse Intake/Indexer/Gate
        JoystickButton(controller1, kB.value).whileHeld(
            ParallelCommandGroup(
                FixedBallMotorSpeed(intake, { -Constants.intakeSpeed }),
                FixedBallMotorSpeed(indexer, { -Constants.indexerSpeed }),
                FixedBallMotorSpeed(gate, { -Constants.gateSpeed })
            )
        )

        // A - Pneumatic Climber Forward
        // X - Pneumatic Climber Backward

        // D-Pad Up - Intake Pneumatic Up TODO
        // D-Pad Down - Intake Pneumatic Down TODO

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