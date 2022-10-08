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
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType

import com.revrobotics.*
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger

import frc.robot.subsystems.*
import frc.robot.commands.*

fun newLiveRecord(name: String): NetworkTableEntry {
    val ntInst = NetworkTableInstance.getDefault()
    val table = ntInst.getTable("liverecord")
    val key = table.getEntry(name)
    return key
}
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    // Controllers
    val primaryController = XboxController(0)
    val secondaryController = XboxController(1)

    // Motors, Solenoids, and Hardware Defintiions
    val motorFrontLeft = CANSparkMax(Constants.driveFrontLeftID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val motorBackLeft = CANSparkMax(Constants.driveBackLeftID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val motorFrontRight = CANSparkMax(Constants.driveFrontRightID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val motorBackRight = CANSparkMax(Constants.driveBackRightID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val intakeSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.intakeSolenoidID.first, Constants.intakeSolenoidID.second)
    val shooterMotor1 = WPI_TalonFX(Constants.shooterLowID)
    val shooterMotor2  = WPI_TalonFX(Constants.shooterHighID)

    // Subsytem Definitions
    val drivetrain = CANSparkMaxDrivetrainSubsystem(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, AHRS())
    val intakePneumatics = SolenoidSubsystem(intakeSolenoid)
    val shooter1 = TalonFXShooterSubsystem(shooterMotor1, 1.0)
    val shooter2 = TalonFXShooterSubsystem(shooterMotor2, 1.0)
    val gate = BallMotorSubsystem(WPI_TalonSRX(Constants.gateID))
    val intakeIndexer = BallMotorSubsystem(WPI_TalonSRX(Constants.intakeID))

    init {
        configureButtonBindings()
        configureAutoOptions()
    }

    /**
     * Controller ([GenericHID], [XboxController]) mapping.
     */
    private fun configureButtonBindings() {
        val runGate = { forward: Boolean ->
            if (forward) {
                FixedBallMotorSpeed(gate) { Constants.gateSpeed }
            } else {
                FixedBallMotorSpeed(gate) { -Constants.gateSpeed }
            }
        }

        val runIntakeIndexer = { forward: Boolean ->
            if (forward) {
                ParallelCommandGroup(
                    FixedBallMotorSpeed(intakeIndexer) { Constants.intakeIndexerSpeed },
                    FixedBallMotorSpeed(gate) { Constants.gateSpeed * .15 }
                )

            } else {
                ParallelCommandGroup(
                    FixedBallMotorSpeed(intakeIndexer) { -Constants.intakeIndexerSpeed },
                    FixedBallMotorSpeed(gate) { -Constants.gateSpeed * .15 }
                )
            }
        }


        val setIntakePneumatics= { forward: Boolean ->
            if (forward) {
                PneumaticCommand(intakePneumatics, DoubleSolenoid.Value.kForward)
            } else {
                PneumaticCommand(intakePneumatics, DoubleSolenoid.Value.kReverse)
            }
        }

//        val turnToAngleOnDpad = { controller: XboxController ->
//            // D-Pad: Turn to fixed angle
//            for (i in 0 until 8) {
//                val angleDeg = 45 * i
//                POVButton(controller, angleDeg).whenHeld(
//                    TurnToAngle(drivetrain, { angleDeg * PI / 180.0 })
//                )
//            }
//        }

        val fixedSpeedShooter = {
            ParallelCommandGroup(
                DualShooterPID(shooter1, shooter2) { DualShootSpeed(Constants.shooterRadPerS, Constants.shooterAdjustRadPerS) },
                ShootBallMotor(shooter1, shooter2, gate, intakeIndexer),
                MaintainAngle(drivetrain)
            )
        }

        drivetrain.defaultCommand = DualStickArcadeDrive(drivetrain, primaryController)

        //turnToAngleOnDpad(primaryController)

        JoystickButton(primaryController, kA.value).whenHeld(runIntakeIndexer(true))
        JoystickButton(primaryController, kB.value).whenHeld(runIntakeIndexer(false))
        JoystickButton(primaryController, kX.value).whenHeld(setIntakePneumatics(true))
        JoystickButton(primaryController, kY.value).whenHeld(setIntakePneumatics(false))

        JoystickButton(secondaryController, kA.value).whenHeld(runIntakeIndexer(true))
        JoystickButton(secondaryController, kB.value).whenHeld(runIntakeIndexer(false))
        JoystickButton(secondaryController, kX.value).whenHeld(setIntakePneumatics(true))
        JoystickButton(secondaryController, kY.value).whenHeld(setIntakePneumatics(false))


        Trigger({ primaryController.rightTriggerAxis > 0.2 }).whileActiveOnce(fixedSpeedShooter())
    }

    var autoCommandChooser: SendableChooser<Command> = SendableChooser()
    private fun configureAutoOptions() {}
    val autonomousCommand: Command get() = autoCommandChooser.selected
}