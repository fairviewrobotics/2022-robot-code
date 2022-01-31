// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.networktables.EntryListenerFlags
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance


private data class ConstantsListener(val func: () -> Unit, val id: Int)

class Constants {
    companion object {
        // ask edward: does allowing all constants to be modified via network tables put a penalty on performance/security/something else
        private val constants = mutableMapOf<String, Double>()

        private fun<T> generateConstantGetter(key: String, default: Double): () -> T {
            constants[key] = default
            return fun(): T {
                return (constants[key] ?: default) as T
            }
        }

        val kDrivetrainFrontLeftPort get() = generateConstantGetter<Int>("kDrivetrainFrontLeftPort", 0.0)()
        val kDrivetrainFrontRightPort get() = generateConstantGetter<Int>("kDrivetrainFrontRightPort", 1.0)()
        val kDrivetrainBackLeftPort get() = generateConstantGetter<Int>("kDrivetrainBottomLeftPort", 2.0)()
        val kDrivetrainBackRightPort get() = generateConstantGetter<Int>("kDrivetrainBottomRightPort", 3.0)()
        
        
        val shooterFFS get() = generateConstantGetter<Double>("shooterFeedForwardS", 3.0)()
        val shooterFFV get() = generateConstantGetter<Double>("shooterFeedForwardV", 3.0)()
        val shooterFFA get() = generateConstantGetter<Double>("shooterFeedForwardA", 3.0)()

        val shooterP get() = generateConstantGetter<Double>("shooterP", 3.0)()
        val shooterI get() = generateConstantGetter<Double>("shooterI", 3.0)()
        val shooterD get() = generateConstantGetter<Double>("shooterD", 3.0)()


        // constants for basic pneumatics

        // SET THESE PROPERLY!
        val compressorID get() = generateConstantGetter<int>("compressorID", -1)()
        val solenoidModuleID get() = generateConstantGetter<int>("solenoidModuleID", -1)()
        val solenoidFChan get() = generateConstantGetter<int>("solenoidFChan", -1)()
        val solenoidRChan get() = generateConstantGetter<int>("solenoidRChan", -1)()

        // constants for flywheel LQR
        val shooterInertia get() = generateConstantGetter<Double>("shooterInertia", 0.0020521)() //  units: kg / m^2
        val shooterGearing get() = generateConstantGetter<Double>("shooterGearing", 1.0)() // output over input, unitless
        val shooterStateStdev get() = generateConstantGetter<Double>("shooterStateStdev", 3.0)()
        val shooterEncStdev get() = generateConstantGetter<Double>("shooterEncStdev", 0.01)()
        val shooterQ get() = generateConstantGetter<Double>("shooterQ", 8.0)()
        val shooterR get() = generateConstantGetter<Double>("shooterR", 12.0)()
        val shooterVolts get() =generateConstantGetter<Double>("shooterVolts", 12.0)()
        val shooterSpinupRadS get() = generateConstantGetter<Double>("shooterSpinupRadS", 3.0)()

        /** NetworkTables Constants Management **/
        private lateinit var table: NetworkTable
        private var listeners = mutableListOf<ConstantsListener>()
        private var listenersId = 0

        init {
            loadConstants()
        }

        /**
         * Load constants from network tables
         */
        fun loadConstants() {
            table = NetworkTableInstance.getDefault().getTable("Constants")
            for (key in constants.keys) {
                if (constants[key] == null) continue

                val entry = table.getEntry(key)

                entry.setDefaultDouble(constants[key] ?: 0.0)
                entry.setPersistent()
                constants[key] = entry.getDouble(constants[key] ?: 0.0)

                entry.addListener({ event ->
                    onNetworkTablesChange()
                }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
            }
        }

        private fun onNetworkTablesChange() {
            /** update map **/
            for (key in constants.keys) {
                val entry = table.getEntry(key)
                constants[key] = entry.getDouble(constants[key] ?: 0.0)
            }
            listeners.map { listen -> listen.func() }
        }

        /**
         * Add a listener for any changes in the Preferences NetworkTable
         * listeners will be called on any change
         * Return an id for the listener that can latter be passed to removeListener
         */
        fun addListener(listener: () -> Unit): Int {
            val id = listenersId++
            listeners.add(ConstantsListener(listener, id))
            return id
        }

        /**
         * Remove a listener with an id created by a call to AddListener
         */
        fun removeListener(id: Int) {
            listeners.removeIf { listen -> listen.id == id }
        }
    }
}