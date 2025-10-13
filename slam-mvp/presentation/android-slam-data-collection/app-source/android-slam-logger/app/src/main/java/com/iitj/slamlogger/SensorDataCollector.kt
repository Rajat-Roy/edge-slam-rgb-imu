package com.iitj.slamlogger

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.flow.receiveAsFlow

data class IMUData(
    val timestamp: Long,
    val sensorType: Int,
    val values: FloatArray,
    val accuracy: Int
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false
        other as IMUData
        return timestamp == other.timestamp && sensorType == other.sensorType
    }

    override fun hashCode(): Int {
        return timestamp.hashCode()
    }
}

class SensorDataCollector(context: Context) : SensorEventListener {
    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

    private val dataChannel = Channel<IMUData>(Channel.UNLIMITED)
    val dataFlow = dataChannel.receiveAsFlow()

    private var isRecording = false

    fun startRecording() {
        if (!isRecording) {
            isRecording = true

            // Register accelerometer with 200Hz sampling rate (5000 microseconds)
            accelerometer?.let {
                sensorManager.registerListener(this, it, 5000)
            }

            // Register gyroscope with 200Hz sampling rate (5000 microseconds)
            gyroscope?.let {
                sensorManager.registerListener(this, it, 5000)
            }
        }
    }

    fun stopRecording() {
        if (isRecording) {
            isRecording = false
            sensorManager.unregisterListener(this)
        }
    }

    override fun onSensorChanged(event: SensorEvent?) {
        if (!isRecording || event == null) return

        val imuData = IMUData(
            timestamp = System.nanoTime(),
            sensorType = event.sensor.type,
            values = event.values.clone(),
            accuracy = event.accuracy
        )

        dataChannel.trySend(imuData)
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        // Handle accuracy changes if needed
    }

    fun isAccelerometerAvailable(): Boolean = accelerometer != null
    fun isGyroscopeAvailable(): Boolean = gyroscope != null
}