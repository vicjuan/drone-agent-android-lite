package com.durendal.droneagent.lite

import android.app.Application
import android.content.Context
import android.util.Log

/**
 * The MSDK is shipped obfuscated and needs its own class loader installed before
 * any SDK class is touched, i.e. inside attachBaseContext. Reflection is used
 * because the loader class only exists inside the aircraft aar; a missing loader
 * must degrade to a clear log line instead of killing the process at startup.
 */
class LiteApplication : Application() {
    override fun attachBaseContext(base: Context) {
        super.attachBaseContext(base)
        try {
            Class.forName("com.cySdkyc.clx.Helper")
                .getMethod("install", Application::class.java)
                .invoke(null, this)
            Log.i(TAG, "DJI runtime loader installed")
        } catch (error: Throwable) {
            Log.w(TAG, "DJI runtime loader is unavailable", error)
        }
    }

    private companion object {
        const val TAG = "LiteApplication"
    }
}
