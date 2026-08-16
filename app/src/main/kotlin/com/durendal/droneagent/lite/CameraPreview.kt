package com.durendal.droneagent.lite

import android.content.Context
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
 * MSDK decodes the video itself: hand `ICameraStreamManager` a Surface and it
 * renders decoded frames into it. That is the same path the main
 * drone-agent-android app uses (`DjiLiveStreamController.attachPreviewSurface`),
 * and the cheapest of the three image paths — no frame listener, no bitmap
 * copy, no encoder.
 *
 * The Surface and the aircraft link come up in an unpredictable order, so the
 * latest surface is remembered and [refresh] re-attaches it once the aircraft
 * is connected. Attaching is idempotent, which is what makes that retry safe.
 */
class CameraPreview(context: Context) {

    val view = SurfaceView(context)

    private var surface: Surface? = null
    private var surfaceWidth = 0
    private var surfaceHeight = 0

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

    /** Re-attaches the current surface; call when the aircraft link changes. */
    fun refresh() {
        if (surface != null) attach()
    }

    fun release() {
        surface?.let(::detach)
        surface = null
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
        }
    }

    private companion object {
        const val TAG = "LiteCameraPreview"
    }
}
