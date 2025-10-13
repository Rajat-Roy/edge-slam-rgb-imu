package com.iitj.slamlogger

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Bundle
import android.util.Log
import androidx.core.content.ContextCompat
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.flow.receiveAsFlow

data class GPSData(
    val timestamp: Long,
    val latitude: Double,
    val longitude: Double,
    val altitude: Double?,
    val accuracy: Float?,
    val speed: Float?,
    val bearing: Float?,
    val provider: String
)

class GPSDataCollector(private val context: Context) : LocationListener {

    private val locationManager = context.getSystemService(Context.LOCATION_SERVICE) as LocationManager
    private val dataChannel = Channel<GPSData>(Channel.UNLIMITED)
    val dataFlow = dataChannel.receiveAsFlow()

    private var isRecording = false
    private var lastLocation: Location? = null

    companion object {
        private const val TAG = "GPSDataCollector"
        private const val MIN_TIME_BETWEEN_UPDATES = 1000L // 1 second
        private const val MIN_DISTANCE_FOR_UPDATE = 1.0f // 1 meter
    }

    fun startRecording(): Boolean {
        if (!isRecording) {
            if (!hasGPSPermissions()) {
                Log.e(TAG, "GPS permissions not granted")
                return false
            }

            if (!isGPSEnabled()) {
                Log.e(TAG, "GPS is not enabled")
                return false
            }

            try {
                isRecording = true

                // Request GPS updates with high accuracy
                locationManager.requestLocationUpdates(
                    LocationManager.GPS_PROVIDER,
                    MIN_TIME_BETWEEN_UPDATES,
                    MIN_DISTANCE_FOR_UPDATE,
                    this
                )

                // Also request network location as backup
                locationManager.requestLocationUpdates(
                    LocationManager.NETWORK_PROVIDER,
                    MIN_TIME_BETWEEN_UPDATES,
                    MIN_DISTANCE_FOR_UPDATE,
                    this
                )

                Log.d(TAG, "GPS recording started")
                return true

            } catch (e: SecurityException) {
                Log.e(TAG, "Security exception when requesting location updates", e)
                isRecording = false
                return false
            } catch (e: Exception) {
                Log.e(TAG, "Exception when starting GPS recording", e)
                isRecording = false
                return false
            }
        }
        return true
    }

    fun stopRecording() {
        if (isRecording) {
            isRecording = false
            locationManager.removeUpdates(this)
            lastLocation = null
            Log.d(TAG, "GPS recording stopped")
        }
    }

    override fun onLocationChanged(location: Location) {
        if (!isRecording) return

        // Filter out duplicate locations
        if (lastLocation != null && location.distanceTo(lastLocation) < MIN_DISTANCE_FOR_UPDATE) {
            return
        }

        lastLocation = location

        val gpsData = GPSData(
            timestamp = System.nanoTime(),
            latitude = location.latitude,
            longitude = location.longitude,
            altitude = if (location.hasAltitude()) location.altitude else null,
            accuracy = if (location.hasAccuracy()) location.accuracy else null,
            speed = if (location.hasSpeed()) location.speed else null,
            bearing = if (location.hasBearing()) location.bearing else null,
            provider = location.provider
        )

        dataChannel.trySend(gpsData)
        Log.d(TAG, "GPS data: ${location.latitude}, ${location.longitude}, accuracy: ${location.accuracy}")
    }

    override fun onProviderEnabled(provider: String) {
        Log.d(TAG, "Location provider enabled: $provider")
    }

    override fun onProviderDisabled(provider: String) {
        Log.d(TAG, "Location provider disabled: $provider")
    }

    @Deprecated("Deprecated in Android 12+")
    override fun onStatusChanged(provider: String?, status: Int, extras: Bundle?) {
        Log.d(TAG, "Location provider status changed: $provider, status: $status")
    }

    private fun hasGPSPermissions(): Boolean {
        return ContextCompat.checkSelfPermission(
            context,
            Manifest.permission.ACCESS_FINE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED && ContextCompat.checkSelfPermission(
            context,
            Manifest.permission.ACCESS_COARSE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED
    }

    private fun isGPSEnabled(): Boolean {
        return locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER) ||
               locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER)
    }

    fun isGPSAvailable(): Boolean {
        return hasGPSPermissions() && isGPSEnabled()
    }

    fun getLastKnownLocation(): Location? {
        return try {
            if (hasGPSPermissions()) {
                locationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER)
                    ?: locationManager.getLastKnownLocation(LocationManager.NETWORK_PROVIDER)
            } else null
        } catch (e: SecurityException) {
            Log.e(TAG, "Security exception when getting last known location", e)
            null
        }
    }
}