package com.durendal.droneagent.lite

import android.content.Context
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.view.Surface
import android.view.SurfaceHolder
import android.view.SurfaceView
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager

/**
 * Live camera view of the aircraft's main camera.
 *
 * MSDK renders decoded frames into the preview Surface and separately supplies
 * throttled RGBA frames to the OpenCV detector. The detector copies only a frame
 * it will process because DJI owns and reuses the callback byte array.
 *
 * The Surface and the aircraft link come up in an unpredictable order, so the
 * latest surface is remembered and [refresh] re-attaches it once the aircraft
 * is connected. Attaching is idempotent, which is what makes that retry safe.
 */
class CameraPreview(
    context: Context,
    private val onRgbaFrame: (ByteArray, Int, Int, Int, Int) -> Unit,
    private val onFrameStreamStale: () -> Unit,
) {

    val view = SurfaceView(context)

    private var surface: Surface? = null
    private var surfaceWidth = 0
    private val mainHandler = Handler(Looper.getMainLooper())
    @Volatile private var monitoringEnabled = false
    @Volatile private var lastFrameAtNanos = 0L
    @Volatile private var frameStreamStale = true
    private var surfaceHeight = 0
    private var frameListenerAttached = false
    private val frameListener = ICameraStreamManager.CameraFrameListener {
            frameData,
            offset,
            length,
            width,
            height,
            format,
        ->
        if (format == ICameraStreamManager.FrameFormat.RGBA_8888) {
            lastFrameAtNanos = System.nanoTime()
            frameStreamStale = false
            onRgbaFrame(frameData, offset, length, width, height)
        }
    }

    private val cameraStreamManager: ICameraStreamManager
        get() = MediaDataCenter.getInstance().cameraStreamManager

    init {
        view.holder.addCallback(
            object : SurfaceHolder.Callback {
                override fun surfaceCreated(holder: SurfaceHolder) = Unit

                override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {
                    if (width <= 0 || height <= 0) return
                    surface = holder.surface
                    surfaceWidth = width
                    surfaceHeight = height
                    attach()
                }

                override fun surfaceDestroyed(holder: SurfaceHolder) {
                    detach(holder.surface)
                    surface = null
                    surfaceWidth = 0
                    surfaceHeight = 0
                }
            },
        )
    }

    /**
     * Keeps the decoded-frame listener alive for the Activity lifetime. The
     * preview Surface can resume before FlightControllerKey.KeyConnection does,
     * so frame recovery must not be gated by the flight-controller link.
     */
    fun refresh() {
        monitoringEnabled = true
        if (surface != null) attach()
        mainHandler.removeCallbacks(frameWatchdog)
        mainHandler.postDelayed(frameWatchdog, FRAME_WATCHDOG_PERIOD_MS)
    }

    fun release() {
        monitoringEnabled = false
        mainHandler.removeCallbacks(frameWatchdog)
        surface?.let(::detach) ?: detachFrameListener()
        surface = null
    }

    private val frameWatchdog = object : Runnable {
        override fun run() {
            if (!monitoringEnabled) return
            val frameAgeNanos = System.nanoTime() - lastFrameAtNanos
            if (frameAgeNanos >= FRAME_STALE_NANOS) {
                if (!frameStreamStale) {
                    frameStreamStale = true
                    Log.w(TAG, "RGBA frame stream stale; reattaching listener")
                    onFrameStreamStale()
                }
                detachFrameListener()
                attach()
            }
            mainHandler.postDelayed(this, FRAME_WATCHDOG_PERIOD_MS)
        }
    }

    private fun attach() {
        val target = surface ?: return
        try {
            val manager = cameraStreamManager
            manager.putCameraStreamSurface(
                ComponentIndexType.LEFT_OR_MAIN,
                target,
                surfaceWidth,
                surfaceHeight,
                ICameraStreamManager.ScaleType.CENTER_INSIDE,
            )
            manager.setKeepAliveDecoding(true)
            manager.enableStream(ComponentIndexType.LEFT_OR_MAIN, true)
            if (!frameListenerAttached) {
                manager.addFrameListener(
                    ComponentIndexType.LEFT_OR_MAIN,
                    ICameraStreamManager.FrameFormat.RGBA_8888,
                    frameListener,
                )
                frameListenerAttached = true
                Log.i(TAG, "RGBA camera frame listener attached")
            }
            Log.i(TAG, "camera surface attached: ${surfaceWidth}x$surfaceHeight")
        } catch (error: Throwable) {
            // Before registration or without an aircraft the manager is not
            // usable yet. That is expected, not fatal: refresh() retries.
            Log.w(TAG, "camera surface attach deferred: $error")
        }
    }

    private fun detach(target: Surface) {
        try {
            cameraStreamManager.removeCameraStreamSurface(target)
            Log.i(TAG, "camera surface detached")
        } catch (error: Throwable) {
            Log.w(TAG, "camera surface detach failed: $error")
        } finally {
            detachFrameListener()
        }
    }

    private fun detachFrameListener() {
        if (!frameListenerAttached) return
        try {
            cameraStreamManager.removeFrameListener(frameListener)
        } catch (error: Throwable) {
            Log.w(TAG, "camera frame listener detach failed: $error")
        } finally {
            frameListenerAttached = false
        }
    }

    private companion object {
        const val TAG = "LiteCameraPreview"
        const val FRAME_WATCHDOG_PERIOD_MS = 1_000L
        const val FRAME_STALE_NANOS = 2_000_000_000L
    }
}
