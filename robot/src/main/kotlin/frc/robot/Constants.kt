// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.networktables.EntryListenerFlags
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance

private data class ConstantsListener(val func: (newValue: Double) -> Unit, val id: Int)

class Constants {
    companion object {
        /* Port IDs */
        // drivetrain (4 motors, 2 on each side)
        val driveFrontLeftID = 1
        val driveBackLeftID = 2
        val driveFrontRightID = 3
        val driveBackRightID = 4

        // climber (2 solenoids and one winch motor)
        val climbSolenoidLeftID = Pair(1, 2)
        val climbSolenoidRightID = Pair(3, 4)
        val climbWinchID = 33

        // shooter (2 spark max to run wheels)
        val shooterLowID = 20
        val shooterHighID = 21

        // intake / indexer / gate (on talon each)
        val intakeID = 9
        val indexerID = 7
        val gateID = 8

        val refreshInterval = 0.02

        private val constants = mutableMapOf<String, Double>()

        /** NetworkTables Constants Management **/
        private val table: NetworkTable = NetworkTableInstance.getDefault().getTable("Constants")
        private var listeners = mutableMapOf<String, MutableList<ConstantsListener>>()
        private var listenersId = 0

        private fun<T> generateConstantGetter(key: String, default: Double): () -> T {
            val entry = table.getEntry(key)

            entry.setDefaultDouble(default)
            entry.setPersistent()
            constants[key] = entry.getDouble(default)

            entry.addListener({ event ->
                onNetworkTablesChange(event.name, event.value.value as Double)
            }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)

            return fun(): T {
                return (constants[key] ?: default) as T
            }
        }

        // drivetrain pid control coefficients
        // these are used for angular control
        val kDrivetrainPidP get() = generateConstantGetter<Double>("kDrivetrainPidP", 1.0)()
        val kDrivetrainPidI get() = generateConstantGetter<Double>("kDrivetrainPidI", 0.0)()
        val kDrivetrainPidD get() = generateConstantGetter<Double>("kDrivetrainPidD", 0.0)()

        // values used for drivetrain commands
        val kDrivetrainSlewRateForwardLimit  get() = generateConstantGetter<Double>("kDrivetrainSlewRateForwardLimit", 10000.0)()
        val kDrivetrainSlewRateRotationLimit  get() = generateConstantGetter<Double>("kDrivetrainSlewRateRotationLimit", 10000.0)()
        val kDrivetrainRegularForwardSpeed get() =  generateConstantGetter<Double>("kDrivetrainRegularForwardSpeed", 5.0)()
        val kDrivetrainRegularRotationSpeed get() = generateConstantGetter<Double>("kDrivetrainRegularRotationSpeed", 0.5)()
        val kDrivetrainFineForwardSpeed get() = generateConstantGetter<Double>("kDrivetrainFineForwardSpeed", 1.25)()
        val kDrivetrainFineRotationSpeed get() = generateConstantGetter<Double>("kDrivetrainFineRotationSpeed", 0.125)()

        val kDrivetrainAngleTolerance get() = generateConstantGetter<Double>("kDrivetrainAngleTolerance", 0.03490658504)()
        val kDrivetrainVelTolerance get() = generateConstantGetter<Double>("kDrivetrainVelTolerance", 0.01745329252)()

        /* Shooter Feed-Forward gains. These gains control the open part (not feedback) of shooter control */
        // baseline (static) gain [V]
        val shooterFFS get() = generateConstantGetter<Double>("shooterFeedForwardS", 0.05)()
        // feedforward velocity gain [V / (rad/s)]
        // this should be approx: 12 V / (free speed in rad/s) = 1 / (kV in V/(rad/s))
        val shooterFFV get() = generateConstantGetter<Double>("shooterFeedForwardV", 0.017961114)()
        // feedforward acceleration gain [V / (rad/s^2)]
        // this should be approx: 12 V * (moment of inertia) /  (stall torque in Nm)
        val shooterFFA get() = generateConstantGetter<Double>("shooterFeedForwardA", 0.000757685)()

        val shooterP get() = generateConstantGetter<Double>("shooterP", 0.02)()
        val shooterI get() = generateConstantGetter<Double>("shooterI", 0.01)()
        val shooterD get() = generateConstantGetter<Double>("shooterD", 0.0)()

        // shooter target rpm
        val shooterRadPerS get() = generateConstantGetter<Double>("shooterRadPerS", 500.0)()
        val shooterAdjustRadPerS get() = generateConstantGetter<Double>("shooterAdjustRadPerS", 0.0)()

        // distance to assume we are from the center of the target if we don't have a measurement (m)
        val shooterDefaultDist = 3.0


        val shooterElevationP get() = generateConstantGetter<Double>("shooterElevationP", 0.5)()
        val shooterElevationI get() = generateConstantGetter<Double>("shooterElevationI", 0.0)()
        val shooterElevationD get() = generateConstantGetter<Double>("shooterElevationD", 0.0)()

        val shooterElevationPosTolerance get() = generateConstantGetter<Double>("shooterElevationPosTolerance", 0.0)()
        val shooterElevationVelocityTolerance get() = generateConstantGetter<Double>("shooterElevationVelocityTolerance", 0.0)()

        // constants for flywheel LQR
        val shooterInertia get() = 0.0020521 //  units: kg / m^2

        // base top speeds for intake, indexer, and gate
        val intakeSpeed = 0.75
        val indexerSpeed = 0.5
        val gateSpeed = -0.75


        private fun onNetworkTablesChange(key: String, value: Double) {
            /** update map **/
            constants[key] = value
            listeners[key]?.map { listen -> listen.func(value) }
        }

        /**
         * Add a listener for any changes in the Preferences NetworkTable
         * listeners will be called on any change
         * Return an id for the listener that can latter be passed to removeListener
         */
        fun addListener(key: String, listener: (value: Double) -> Unit): Int {
            val id = listenersId++
            if(listeners[key] == null) {
                listeners[key] = mutableListOf()
            }
            listeners[key]?.add(ConstantsListener(listener, id))
            return id
        }

        /**
         * Remove a listener with an id created by a call to AddListener
         */
        fun removeListener(key: String, id: Int) {
            listeners[key]?.removeIf { listen -> listen.id == id }
        }
    }
}