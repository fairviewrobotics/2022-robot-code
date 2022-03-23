// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import CheckVisionOrRumble
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.XboxController.Button.*
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj.DigitalInput

import com.revrobotics.*
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj2.command.button.POVButton
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
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
    val winchMotor = CANSparkMax(Constants.climbWinchID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val winch = WinchSubsystem(winchMotor, DigitalInput(0), DigitalInput(1), true)


    val climbSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.climbSolenoidID.first,Constants.climbSolenoidID.second) 
    val intakeSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.intakeSolenoidID.first, Constants.intakeSolenoidID.second)
    val climbPneumatics = SolenoidSubsystem(climbSolenoid)
    val intakePneumatics = SolenoidSubsystem(intakeSolenoid)

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

    // auto command chooser
    var autoCommandChooser: SendableChooser<Command> = SendableChooser()

    init {
        configureButtonBindings()
        configureAutoOptions()
    }

    /**
     * Controller ([GenericHID], [XboxController]) mapping.
     */
    private fun configureButtonBindings() {
        // See https://blackknightsrobotics.slack.com/files/UML602T96/F0377HEMXU3/image_from_ios.jpg For the control scheme.

        // PRIMARY DRIVER
        drivetrain.defaultCommand = DualStickArcadeDrive(drivetrain, controller0)

        // LT - Vision Lineup to ball
        Trigger { controller0.leftTriggerAxis > 0.2 }.whileActiveOnce (
            ParallelCommandGroup(
                TurnToBall(controller0, drivetrain),
                FixedBallMotorSpeed(intake, { Constants.intakeSpeed } )
            )
        )

        // LB - Vision Lineup to High Goal
        JoystickButton(controller0, kLeftBumper.value).whenHeld(
            SequentialCommandGroup(
                CheckVisionOrRumble(controller0),
                TurnToHighGoal(drivetrain)
            )
        )

        // RT - Set Manual Shooting Power
        Trigger { controller0.rightTriggerAxis > 0.2 }.whileActiveOnce (
            ParallelCommandGroup(
                FixedShooterSpeed(shooter1, { controller0.rightTriggerAxis }),
                FixedShooterSpeed(shooter2, { -1.0 * controller0.rightTriggerAxis })
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

        // A - deploy intake pneumatic
        JoystickButton(controller0, kA.value).whenHeld(
            PneumaticCommand(intakePneumatics, DoubleSolenoid.Value.kForward)
        )

        // B - Run Intake, Indexer, and, until color sensor detects, Gate
        JoystickButton(controller0, kB.value).whenHeld(
            ParallelCommandGroup(
                FixedBallMotorSpeed(intake, { Constants.intakeSpeed }),
                FixedBallMotorSpeed(indexer, { Constants.indexerSpeed })
            )
        )
        
        // Y - retract intake pneumatic
        JoystickButton(controller0, kY.value).whenHeld(
            PneumaticCommand(intakePneumatics, DoubleSolenoid.Value.kReverse)
        )

        // B - Run Intake
        // TODO: conflict!
        /* 
        JoystickButton(controller0, kB.value).whenHeld(
            FixedBallMotorSpeed(intake, { Constants.intakeSpeed })
        )
        */

        // D-Pad: Turn to fixed angle
        for (i in 0 until 8) {
            val angleDeg = 45 * i
            POVButton(controller0, angleDeg).whenHeld(
                TurnToAngle(drivetrain, { angleDeg * PI / 180.0 })
            )
        }

        // SECONDARY DRIVER

        // LT - Climber Down
        Trigger({ controller1.leftTriggerAxis > 0.2 }).whileActiveOnce (
            FixedWinchVoltage(winch, { 5.0 * controller1.leftTriggerAxis })
        )

        // RT - Climber Up
        Trigger({ controller1.rightTriggerAxis > 0.2 }).whileActiveOnce (
            FixedWinchVoltage(winch, { -5.0 * controller1.rightTriggerAxis })
        )

        // LB - Auto Climb
        JoystickButton(controller1, kLeftBumper.value).whenHeld(
            AutoClimb(winch, climbPneumatics)
        )

        // POV button climber control
        // Right - run climber all the way up
        POVButton(controller1, 0).whenHeld(
            WinchPIDCommand(winch, { Constants.elevatorMaxPos })
        )
        // Left - run climber all the way down
        POVButton(controller1, 180).whenHeld(
            WinchPIDCommand(winch, { Constants.elevatorMinPos })
        )
        // Left-up - run climber half way up
        POVButton(controller1, 135).whenHeld(
            WinchPIDCommand(winch, { Constants.elevatorMaxPos * 0.5 })
        )

        // RB - Run Intake/Indexer/Gate
        // TODO: Conflict!
        /* 
        Trigger { controller1.rightTriggerAxis > 0.2 }.whileActiveOnce(
            ParallelCommandGroup(
                FixedBallMotorSpeed(intake, { Constants.intakeSpeed }),
                FixedBallMotorSpeed(indexer, { Constants.indexerSpeed }),
                GateSensored(gate, { Constants.gateSpeed }, colorSensor)
            )
        )
        */

        // run indexer rejection on Y of secondary controller
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
        
        JoystickButton(controller1, kA.value).whenHeld(
            PneumaticCommand(climbPneumatics, DoubleSolenoid.Value.kForward)

        )

        // X - Pneumatic Climber Backward
        JoystickButton(controller1, kX.value).whenHeld(
            PneumaticCommand(climbPneumatics, DoubleSolenoid.Value.kReverse)
        )

        // D-Pad Up - Intake Pneumatic Up
        POVButton(controller1, 90).whenHeld(
            PneumaticCommand(intakePneumatics, DoubleSolenoid.Value.kReverse)
        )

        // D-Pad Down - Intake Pneumatic Down
        POVButton(controller1, 270).whenHeld(
            PneumaticCommand(intakePneumatics, DoubleSolenoid.Value.kForward)
        )

    }

    private fun configureAutoOptions() {
        // Drive forwards for 1.5s to clear tarmac [2pt]
        autoCommandChooser.addOption("Drive forward [2pt]",
            DrivetrainPIDCommand(drivetrain) {
                DifferentialDriveWheelSpeeds(Constants.kDrivetrainFineForwardSpeed, Constants.kDrivetrainFineForwardSpeed)
            }.withTimeout(1.5)
        )
        // Shoot based on vision [4pt] then drive backwards to clear tarmac [2pt]
        autoCommandChooser.addOption("Shoot Vision + Drive forward [6pt]",
            SequentialCommandGroup(
                // shoot based on vision
                ShootVision(drivetrain, shooter1, shooter2, gate, indexer, controller0, true).withTimeout(8.0),
                // shoot at default distance (just in case vision did not work)
                ShootDefaultDistance(shooter1, shooter2, gate, indexer, true).withTimeout(3.0),
                // drive off tarmac
                DrivetrainPIDCommand(drivetrain) {
                    DifferentialDriveWheelSpeeds(Constants.kDrivetrainFineForwardSpeed, Constants.kDrivetrainFineForwardSpeed)
                }.withTimeout(1.5)
            )
        )
    }


    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() = autoCommandChooser.selected
}