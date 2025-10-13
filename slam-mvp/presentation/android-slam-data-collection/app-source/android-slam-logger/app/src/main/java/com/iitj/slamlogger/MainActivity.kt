package com.iitj.slamlogger

import android.Manifest
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.ImageFormat
import android.graphics.Matrix
import android.graphics.Rect
import android.graphics.YuvImage
import android.os.Bundle
import android.util.Log
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.camera.core.*
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.core.content.ContextCompat
import androidx.lifecycle.lifecycleScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.io.ByteArrayOutputStream
import java.io.FileOutputStream
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors

class MainActivity : AppCompatActivity() {

    private lateinit var previewView: PreviewView
    private lateinit var recordButton: Button
    private lateinit var stopButton: Button
    private lateinit var statusText: TextView
    private lateinit var frameCountText: TextView
    private lateinit var imuCountText: TextView

    private lateinit var cameraExecutor: ExecutorService
    private lateinit var sensorCollector: SensorDataCollector
    private lateinit var dataLogger: DataLogger

    private var imageAnalysis: ImageAnalysis? = null
    private var isRecording = false
    private var frameNumber = 0

    private val requestPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        val allGranted = permissions.entries.all { it.value }
        if (allGranted) {
            startCamera()
        } else {
            Toast.makeText(this, "Permissions required for camera and storage", Toast.LENGTH_SHORT).show()
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // Initialize views
        previewView = findViewById(R.id.previewView)
        recordButton = findViewById(R.id.recordButton)
        stopButton = findViewById(R.id.stopButton)
        statusText = findViewById(R.id.statusText)
        frameCountText = findViewById(R.id.frameCountText)
        imuCountText = findViewById(R.id.imuCountText)

        // Initialize components
        cameraExecutor = Executors.newSingleThreadExecutor()
        sensorCollector = SensorDataCollector(this)
        dataLogger = DataLogger(this)

        // Set up button listeners
        recordButton.setOnClickListener { startRecording() }
        stopButton.setOnClickListener { stopRecording() }

        // Check permissions
        checkPermissions()

        // Start collecting IMU data flow
        lifecycleScope.launch {
            sensorCollector.dataFlow.collect { imuData ->
                if (isRecording) {
                    dataLogger.logIMUData(imuData)
                    withContext(Dispatchers.Main) {
                        imuCountText.text = "IMU samples: ${dataLogger.getIMUCount()}"
                    }
                }
            }
        }
    }

    private fun checkPermissions() {
        val requiredPermissions = arrayOf(
            Manifest.permission.CAMERA,
            Manifest.permission.WRITE_EXTERNAL_STORAGE,
            Manifest.permission.READ_EXTERNAL_STORAGE
        )

        val missingPermissions = requiredPermissions.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }

        if (missingPermissions.isEmpty()) {
            startCamera()
        } else {
            requestPermissionLauncher.launch(requiredPermissions)
        }
    }

    private fun startCamera() {
        val cameraProviderFuture = ProcessCameraProvider.getInstance(this)

        cameraProviderFuture.addListener({
            val cameraProvider = cameraProviderFuture.get()

            // Preview
            val preview = Preview.Builder().build().also {
                it.setSurfaceProvider(previewView.surfaceProvider)
            }

            // Image analysis for frame capture
            imageAnalysis = ImageAnalysis.Builder()
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .build()
                .also {
                    it.setAnalyzer(cameraExecutor, FrameAnalyzer())
                }

            // Select back camera
            val cameraSelector = CameraSelector.DEFAULT_BACK_CAMERA

            try {
                cameraProvider.unbindAll()
                cameraProvider.bindToLifecycle(
                    this, cameraSelector, preview, imageAnalysis
                )
            } catch (exc: Exception) {
                Log.e("CameraX", "Use case binding failed", exc)
            }

        }, ContextCompat.getMainExecutor(this))
    }

    private fun startRecording() {
        lifecycleScope.launch {
            try {
                val sessionId = dataLogger.startSession()
                sensorCollector.startRecording()

                withContext(Dispatchers.Main) {
                    isRecording = true
                    frameNumber = 0
                    recordButton.isEnabled = false
                    stopButton.isEnabled = true
                    statusText.text = "Recording... Session: $sessionId"
                }

                Log.d("SLAM", "Started recording session: $sessionId")
            } catch (e: Exception) {
                Log.e("SLAM", "Failed to start recording", e)
                withContext(Dispatchers.Main) {
                    Toast.makeText(this@MainActivity, "Failed to start recording", Toast.LENGTH_SHORT).show()
                }
            }
        }
    }

    private fun stopRecording() {
        lifecycleScope.launch {
            try {
                isRecording = false
                sensorCollector.stopRecording()
                val metadata = dataLogger.endSession()

                withContext(Dispatchers.Main) {
                    recordButton.isEnabled = true
                    stopButton.isEnabled = false
                    statusText.text = "Recording stopped. Frames: ${metadata.totalFrames}, IMU: ${metadata.totalImuSamples}"
                }

                Log.d("SLAM", "Recording stopped. Session saved to: ${dataLogger.getSessionDir()}")
                Log.d("SLAM", "Metadata: $metadata")
            } catch (e: Exception) {
                Log.e("SLAM", "Failed to stop recording", e)
            }
        }
    }

    private inner class FrameAnalyzer : ImageAnalysis.Analyzer {
        override fun analyze(image: ImageProxy) {
            if (isRecording) {
                lifecycleScope.launch(Dispatchers.IO) {
                    try {
                        saveFrame(image)
                    } catch (e: Exception) {
                        Log.e("SLAM", "Failed to save frame", e)
                    } finally {
                        image.close()
                    }
                }
            } else {
                image.close()
            }
        }

        private suspend fun saveFrame(image: ImageProxy) {
            val timestamp = System.nanoTime()
            frameNumber++

            // Convert ImageProxy to JPEG
            val bitmap = imageProxyToBitmap(image)
            val imageFile = dataLogger.getImageFile(frameNumber)

            imageFile?.let { file ->
                FileOutputStream(file).use { output ->
                    bitmap.compress(Bitmap.CompressFormat.JPEG, 90, output)
                }

                // Log camera frame data
                dataLogger.logCameraFrame(
                    timestamp = timestamp,
                    frameNumber = frameNumber,
                    fileName = file.name,
                    width = bitmap.width,
                    height = bitmap.height
                )

                withContext(Dispatchers.Main) {
                    frameCountText.text = "Frames: ${dataLogger.getFrameCount()}"
                }
            }
        }

        private fun imageProxyToBitmap(image: ImageProxy): Bitmap {
            val yBuffer = image.planes[0].buffer
            val uBuffer = image.planes[1].buffer
            val vBuffer = image.planes[2].buffer

            val ySize = yBuffer.remaining()
            val uSize = uBuffer.remaining()
            val vSize = vBuffer.remaining()

            val nv21 = ByteArray(ySize + uSize + vSize)

            yBuffer.get(nv21, 0, ySize)
            vBuffer.get(nv21, ySize, vSize)
            uBuffer.get(nv21, ySize + vSize, uSize)

            val yuvImage = YuvImage(nv21, ImageFormat.NV21, image.width, image.height, null)
            val out = ByteArrayOutputStream()
            yuvImage.compressToJpeg(Rect(0, 0, yuvImage.width, yuvImage.height), 90, out)
            val yuv = out.toByteArray()
            return BitmapFactory.decodeByteArray(yuv, 0, yuv.size)
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        cameraExecutor.shutdown()
        sensorCollector.stopRecording()
    }
}