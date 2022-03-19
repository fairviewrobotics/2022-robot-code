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
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj.DigitalInput
import kotlin.math.*

import com.revrobotics.*
import edu.wpi.first.wpilibj2.command.button.Trigger


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
    val winchMotor = CANSparkMax(Constants.climbWinchID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val winch = WinchSubsystem(winchMotor, DigitalInput(0), DigitalInput(1))

    val climbSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.climbSolenoidID.first,Constants.climbSolenoidID.second) 
    val intakeSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.intakeSolenoidID.first, Constants.intakeSolenoidID.second)
    val climbPneumatics = SolenoidSubsystem(climbSolenoid)
    val intakePneumatics = SolenoidSubsystem(intakeSolenoid)
    

    // shooter
    val shooterMotor1 = WPI_TalonFX(Constants.shooterLowID)
    val shooterMotor2  = WPI_TalonFX(Constants.shooterHighID)
    val shooter1 = TalonFXShooterSubsystem(shooterMotor1, 1.0)
    val shooter2 = TalonFXShooterSubsystem(shooterMotor2, -1.0)

    // todo: set values!
    val shooterElevationEncoder = Encoder(Constants.shooterElevationEncoderIDA, Constants.shooterElevationEncoderIDB) 
    val shooterElevation = ShooterElevationSubsystem(WPI_TalonSRX(Constants.shooterElevationMotorID))

    // intake / indexer / gate
    val intake = BallMotorSubsystem(WPI_TalonSRX(Constants.intakeID))
    val indexer = BallMotorSubsystem(WPI_TalonSRX(Constants.indexerID))
    val gate = BallMotorSubsystem(WPI_TalonSRX(Constants.gateID))
    
    // simultaneous pneumatics push and pull
    // todo: remove below: unnecessary
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
        // run shooter + vision on controller0 right bumper
        JoystickButton(controller0, kRightBumper.value).whenHeld(
            ShootVision(drivetrain, shooter1, shooter2, gate, indexer, controller0)
        )
        shooter2.defaultCommand = ShooterBangBang(shooter2, {
            if (controller0.getRightTriggerAxis() > 0.5) -1.0 * Constants.shooterRadPerS else(
                0.0
            )
        })

        // run shooter without vision on controller0 right trigger
        Trigger { controller0.rightTriggerAxis > 0.2 }.whileActiveOnce(
            ShootDefaultDistance(shooter1, shooter2, gate, indexer)
        )
        

        // raise/lower intake on X/A
        JoystickButton(controller0, kX.value).whenHeld(
            PneumaticCommand(intakePneumatics, DoubleSolenoid.Value.kForward)
            
        )

        JoystickButton(controller0, kA.value).whenHeld(
            PneumaticCommand(intakePneumatics, DoubleSolenoid.Value.kReverse)
        )

        // raise / lower climber
        winch.defaultCommand = DebugClimbingCommand(winch, controller1)
        //winch.defaultCommand = WinchPIDCommand(winch, controller1)
        //winch.defaultCommand = LimitedWinchCommand(winch, { controller0.rightY })
        

        // raise / lower climber pneumatic component
        JoystickButton(controller0, kY.value).whenHeld(
            PneumaticCommand(climbPneumatics, DoubleSolenoid.Value.kForward)
            
        )

        JoystickButton(controller0, kB.value).whenHeld(
            PneumaticCommand(climbPneumatics, DoubleSolenoid.Value.kReverse)
        )


        // run intake + indexer on RB
        JoystickButton(controller0, kRightBumper.value).whenHeld(
            ParallelCommandGroup(
                FixedBallMotorSpeed(intake, { Constants.intakeSpeed }),
                FixedBallMotorSpeed(indexer, { Constants.indexerSpeed })
            )
        )

        // raise / lower shooter: rudimentarily discrete for now

        var shooterDist = 100.0

        shooterElevation.defaultCommand = ElevationCommand(shooterElevation, fun() : Double{
            if (controller0.getLeftBumper()){
                shooterDist = shooterDist - 100
                return max(shooterDist, 0.0)
            } else if (controller0.getLeftTriggerAxis() > 0.5){
                shooterDist = shooterDist + 100
                return min(shooterDist, Constants.elevationEncoderMax)
            } else{
                return shooterDist
            }
        }, shooterElevationEncoder)

        // run indexer rejection on Y of secondary controller
        JoystickButton(controller1, kY.value).whenHeld(
            ParallelCommandGroup(
                FixedBallMotorSpeed(gate, { -1.0 * Constants.gateSpeed }),
                FixedBallMotorSpeed(indexer, { -1.0 * Constants.indexerSpeed }),
            )
        )

        drivetrain.defaultCommand = ArcadeDrive(drivetrain, controller0)
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