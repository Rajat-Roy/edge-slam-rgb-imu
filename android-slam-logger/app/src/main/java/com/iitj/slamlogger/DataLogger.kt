package com.iitj.slamlogger

import android.content.Context
import android.os.Environment
import com.google.gson.Gson
import com.google.gson.GsonBuilder
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import java.io.File
import java.io.FileWriter
import java.text.SimpleDateFormat
import java.util.*

data class CameraFrame(
    val timestamp: Long,
    val frameNumber: Int,
    val fileName: String,
    val width: Int,
    val height: Int
)

data class SessionMetadata(
    val sessionId: String,
    val startTime: Long,
    val endTime: Long,
    val totalFrames: Int,
    val totalImuSamples: Int,
    val totalGpsSamples: Int,
    val frameRate: Double,
    val imuSampleRate: Double,
    val gpsSampleRate: Double
)

class DataLogger(private val context: Context) {
    private val gson: Gson = GsonBuilder().setPrettyPrinting().create()
    private var sessionId: String = ""
    private var sessionDir: File? = null
    private var imuWriter: FileWriter? = null
    private var cameraWriter: FileWriter? = null
    private var gpsWriter: FileWriter? = null

    private var startTime: Long = 0
    private var frameCount = 0
    private var imuCount = 0
    private var gpsCount = 0

    suspend fun startSession(): String = withContext(Dispatchers.IO) {
        val timestamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault()).format(Date())
        sessionId = "slam_session_$timestamp"

        // Create session directory
        val appDir = File(context.getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS), "SLAMLogger")
        sessionDir = File(appDir, sessionId)
        sessionDir?.mkdirs()

        // Create subdirectories
        File(sessionDir, "images").mkdirs()
        File(sessionDir, "data").mkdirs()

        // Initialize data files
        imuWriter = FileWriter(File(sessionDir, "data/imu_data.jsonl"))
        cameraWriter = FileWriter(File(sessionDir, "data/camera_frames.jsonl"))
        gpsWriter = FileWriter(File(sessionDir, "data/gps_data.jsonl"))

        startTime = System.currentTimeMillis()
        frameCount = 0
        imuCount = 0
        gpsCount = 0

        sessionId
    }

    suspend fun logIMUData(imuData: IMUData) = withContext(Dispatchers.IO) {
        imuWriter?.let { writer ->
            val json = gson.toJson(imuData)
            writer.appendLine(json)
            writer.flush()
            imuCount++
        }
    }

    suspend fun logCameraFrame(timestamp: Long, frameNumber: Int, fileName: String, width: Int, height: Int) = withContext(Dispatchers.IO) {
        val cameraFrame = CameraFrame(timestamp, frameNumber, fileName, width, height)
        cameraWriter?.let { writer ->
            val json = gson.toJson(cameraFrame)
            writer.appendLine(json)
            writer.flush()
            frameCount++
        }
    }

    suspend fun logGPSData(gpsData: GPSData) = withContext(Dispatchers.IO) {
        gpsWriter?.let { writer ->
            val json = gson.toJson(gpsData)
            writer.appendLine(json)
            writer.flush()
            gpsCount++
        }
    }

    suspend fun endSession() = withContext(Dispatchers.IO) {
        val endTime = System.currentTimeMillis()
        val duration = (endTime - startTime) / 1000.0

        // Close writers
        imuWriter?.close()
        cameraWriter?.close()
        gpsWriter?.close()

        // Create session metadata
        val metadata = SessionMetadata(
            sessionId = sessionId,
            startTime = startTime,
            endTime = endTime,
            totalFrames = frameCount,
            totalImuSamples = imuCount,
            totalGpsSamples = gpsCount,
            frameRate = frameCount / duration,
            imuSampleRate = imuCount / duration,
            gpsSampleRate = gpsCount / duration
        )

        // Save metadata
        val metadataFile = File(sessionDir, "session_metadata.json")
        FileWriter(metadataFile).use { writer ->
            gson.toJson(metadata, writer)
        }

        metadata
    }

    fun getImageFile(frameNumber: Int): File? {
        return sessionDir?.let { dir ->
            File(dir, "images/frame_${String.format("%06d", frameNumber)}.jpg")
        }
    }

    fun getSessionDir(): File? = sessionDir

    fun getFrameCount(): Int = frameCount
    fun getIMUCount(): Int = imuCount
    fun getGPSCount(): Int = gpsCount
}