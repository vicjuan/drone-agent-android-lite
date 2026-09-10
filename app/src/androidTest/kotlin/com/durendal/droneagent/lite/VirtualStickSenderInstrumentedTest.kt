package com.durendal.droneagent.lite

import androidx.test.ext.junit.runners.AndroidJUnit4
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam
import dji.v5.common.callback.CommonCallbacks
import dji.v5.manager.interfaces.IVirtualStickManager
import java.lang.reflect.Proxy
import java.util.Collections
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith

@RunWith(AndroidJUnit4::class)
class VirtualStickSenderInstrumentedTest {
    @Test
    fun releaseWaitsForEarlierSubmissionBeforeSendingFinalNeutral() {
        val motionEntered = CountDownLatch(1)
        val finishMotion = CountDownLatch(1)
        val releaseStarted = CountDownLatch(1)
        val releaseCompleted = CountDownLatch(1)
        val submitted = Collections.synchronizedList(mutableListOf<Double>())
        val session = VirtualStickSession(onStatus = {}, manager = manager { param ->
            if (param.roll != 0.0) {
                motionEntered.countDown()
                check(finishMotion.await(5, TimeUnit.SECONDS))
            }
            submitted.add(param.roll)
        })
        var releaseThread: Thread? = null
        try {
            session.enable {}
            session.setHorizontalVelocity(0.3, 0.0, System.nanoTime() + 2_000_000_000L)
            assertTrue(motionEntered.await(2, TimeUnit.SECONDS))
            releaseThread = Thread {
                releaseStarted.countDown()
                session.disable { releaseCompleted.countDown() }
            }.apply { start() }
            assertTrue(releaseStarted.await(2, TimeUnit.SECONDS))
            assertFalse("release must not overtake an unfinished motion submission", releaseCompleted.await(200, TimeUnit.MILLISECONDS))
            finishMotion.countDown()
            assertTrue(releaseCompleted.await(2, TimeUnit.SECONDS))
            assertEquals(0.0, submitted.last(), 0.0)
        } finally {
            finishMotion.countDown()
            releaseThread?.join(2_000L)
            session.close()
        }
    }

    @Test
    fun expiredHorizontalCommandBecomesNeutralWithoutChangingYawOrClimb() {
        val motionSent = CountDownLatch(1)
        val neutralAfterMotion = CountDownLatch(1)
        val session = VirtualStickSession(onStatus = {}, manager = manager { param ->
            if (param.roll != 0.0) motionSent.countDown()
            if (motionSent.count == 0L && param.roll == 0.0) {
                assertEquals(0.12, param.verticalThrottle, 0.0)
                assertEquals(30.0, param.yaw, 0.0)
                neutralAfterMotion.countDown()
            }
        })
        try {
            session.enable {}
            session.setClimbRate(0.12)
            session.setYawHeading(30.0)
            session.setHorizontalVelocity(0.3, 0.0, System.nanoTime() + 300_000_000L)
            assertTrue(motionSent.await(2, TimeUnit.SECONDS))
            assertTrue(neutralAfterMotion.await(2, TimeUnit.SECONDS))
        } finally {
            session.close()
        }
    }

    private fun manager(submit: (VirtualStickFlightControlParam) -> Unit): IVirtualStickManager =
        Proxy.newProxyInstance(
            IVirtualStickManager::class.java.classLoader,
            arrayOf(IVirtualStickManager::class.java),
        ) { _, method, arguments ->
            when (method.name) {
                "enableVirtualStick", "disableVirtualStick" ->
                    (arguments!![0] as CommonCallbacks.CompletionCallback).onSuccess()
                "sendVirtualStickAdvancedParam" -> submit(arguments!![0] as VirtualStickFlightControlParam)
            }
            null
        } as IVirtualStickManager
}
