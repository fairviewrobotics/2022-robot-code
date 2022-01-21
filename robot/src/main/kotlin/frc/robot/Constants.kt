// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.networktables.EntryListenerFlags
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance


class Units {
    companion object {
        // convert rpm to rad/s
        fun rpmToRadPerSec(rpm: Double) = rpm / 60.0 * 2.0 * Math.PI

        val refreshInterval = 0.02
    }
}

private data class ConstantsListener(val func: (newValue: Double) -> Unit, val id: Int)

class Constants {
    companion object {
        // ask edward: does allowing all constants to be modified via network tables put a penalty on performance/security/something else
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

        val kDrivetrainFrontLeftPort get() = generateConstantGetter<Int>("kDrivetrainFrontLeftPort", 0.0)()
        val kDrivetrainFrontRightPort get() = generateConstantGetter<Int>("kDrivetrainFrontRightPort", 1.0)()
        val kDrivetrainBackLeftPort get() = generateConstantGetter<Int>("kDrivetrainBottomLeftPort", 2.0)()
        val kDrivetrainBackRightPort get() = generateConstantGetter<Int>("kDrivetrainBottomRightPort", 3.0)()
        
        
        val shooterFFS get() = generateConstantGetter<Double>("shooterFeedForwardS", 1.0)()
        val shooterFFV get() = generateConstantGetter<Double>("shooterFeedForwardV", 0.0)()
        val shooterFFA get() = generateConstantGetter<Double>("shooterFeedForwardA", 0.0)()

        val shooterP get() = generateConstantGetter<Double>("shooterP", 0.5)()
        val shooterI get() = generateConstantGetter<Double>("shooterI", 0.0)()
        val shooterD get() = generateConstantGetter<Double>("shooterD", 0.0)()

        // constants for flywheel LQR
        val shooterInertia get() = generateConstantGetter<Double>("shooterInertia", 0.0020521)() //  units: kg / m^2
        val shooterGearing get() = generateConstantGetter<Double>("shooterGearing", 1.0)() // output over input, unitless
        val shooterStateStdev get() = generateConstantGetter<Double>("shooterStateStdev", 3.0)()
        val shooterEncStdev get() = generateConstantGetter<Double>("shooterEncStdev", 0.01)()
        val shooterQ get() = generateConstantGetter<Double>("shooterQ", 8.0)()
        val shooterR get() = generateConstantGetter<Double>("shooterR", 12.0)()
        val shooterVolts get() =generateConstantGetter<Double>("shooterVolts", 12.0)()

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