package com.durendal.droneagent.lite

import android.content.Context
import android.util.Log
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.Executors

/**
 * Append-only flight log on the device's own storage.
 *
 * logcat is not usable as evidence here: the MSDK logs so heavily that this app's
 * lines are evicted from the ring buffer within seconds, which on 2026-08-17 lost
 * the record of two flights. A file the app owns cannot be evicted by another
 * component's chatter.
 *
 * Writes go to a single background thread so a 10 Hz control loop never blocks on
 * storage, and every line carries a wall-clock timestamp so it can be lined up
 * with what the operator saw.
 */
class FlightLog(context: Context) {

    private val file = File(context.getExternalFilesDir(null), FILE_NAME)
    private val writer = Executors.newSingleThreadExecutor { runnable ->
        Thread(runnable, "LiteFlightLog").apply { isDaemon = true }
    }
    private val stamp = SimpleDateFormat("MM-dd HH:mm:ss.SSS", Locale.US)

    val path: String get() = file.absolutePath

    fun write(line: String) {
        val text = "${stamp.format(Date())} $line\n"
        Log.i(TAG, line)
        writer.execute {
            runCatching { file.appendText(text) }
                .onFailure { Log.w(TAG, "flight log write failed", it) }
        }
    }

    fun close() = writer.shutdown()

    private companion object {
        const val TAG = "LiteFlightLog"
        const val FILE_NAME = "flight-log.txt"
    }
}
