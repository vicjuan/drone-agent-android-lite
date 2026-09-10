package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class VelocityReadDiagnosticsTest {
    @Test
    fun `pending reads never overlap and unchanged responses remain independent records`() {
        val f = Fixture()
        f.probe.start("first", "CRUISE", 0.0)
        f.probe.tick()
        f.now = 50_000_000L
        f.reply(0)
        f.now = 199_999_999L
        f.probe.tick()
        assertEquals(1, f.callbacks.size)
        f.now = 200_000_000L
        f.probe.tick()
        f.now = 2_000_000_000L
        f.probe.tick()
        assertEquals(2, f.callbacks.size)
        f.reply(1)
        assertEquals(2, f.results.size)
        assertEquals("1", f.results[0]["requestId"])
        assertEquals("2", f.results[1]["requestId"])
        assertEquals("1800.0", f.results[1]["roundTripMs"])
        assertEquals("0", f.results[1]["listenerReceivedAtNanos"])
        assertEquals("2000.0", f.results[1]["listenerAgeMs"])
        assertEquals("0.3", f.results[1]["alongRawMps"])
    }

    @Test
    fun `late result belongs to stopped attempt and duplicate cannot release a new request`() {
        val f = Fixture()
        f.probe.start("old", "CRUISE", 0.0)
        f.probe.tick()
        f.probe.stop()
        f.now = 1_000_000_000L
        f.probe.start("new", "CRUISE", 0.0)
        f.probe.tick()
        assertEquals(1, f.callbacks.size)
        f.reply(0)
        assertEquals("old", f.results.single()["attemptId"])
        assertEquals("false", f.results.single()["scopeActive"])
        f.probe.tick()
        f.reply(0)
        f.now = 2_000_000_000L
        f.probe.tick()
        assertEquals(2, f.callbacks.size)
        assertEquals(1, f.results.size)
        f.reply(1)
        assertEquals("new", f.results.last()["attemptId"])
        assertEquals("true", f.results.last()["scopeActive"])
    }

    @Test
    fun `phase transition during read keeps both request and response phase`() {
        val f = Fixture()
        f.probe.start("flight", "CRUISE", -90.0)
        f.probe.tick()
        f.probe.updatePhase("BRAKING")
        f.callbacks[0](VelocityReadDiagnostics.Velocity(0.0, -0.4, 0.0), null)
        val result = f.results.single()
        assertEquals("CRUISE", result["phase"])
        assertEquals("BRAKING", result["responsePhase"])
        assertEquals(0.4, result.getValue("alongRawMps").toDouble(), 1e-12)
    }

    @Test
    fun `read failure and nonfinite value are not reported as valid zero speed`() {
        val f = Fixture()
        f.probe.start("flight", "CRUISE", null)
        f.probe.tick()
        f.callbacks[0](null, "SDK timeout")
        assertEquals("false", f.results[0]["success"])
        assertEquals("false", f.results[0]["valueValid"])
        assertEquals("SDK_timeout", f.results[0]["error"])
        assertEquals("null", f.results[0]["groundSpeed"])
        f.now = 200_000_000L
        f.probe.tick()
        f.callbacks[1](VelocityReadDiagnostics.Velocity(Double.NaN, 0.0, 0.0), null)
        assertEquals("true", f.results[1]["success"])
        assertEquals("false", f.results[1]["valueValid"])
        assertEquals("NON_FINITE_VALUE", f.results[1]["error"])
        assertEquals("null", f.results[1]["alongRawMps"])
        assertEquals("null", f.results[1]["groundSpeed"])
    }

    @Test
    fun `synchronous SDK exception is recorded without wedging future sampling`() {
        var now = 0L
        val records = mutableListOf<String>()
        val probe = VelocityReadDiagnostics(
            readVelocity = { throw IllegalStateException("not ready") },
            listenerSnapshot = { null },
            record = { event, _, details -> if (event == "velocity_read_result") records += details },
            clock = { now },
        )
        probe.start("flight", "ACQUIRING", null)
        probe.tick()
        now = 200_000_000L
        probe.tick()
        assertEquals(2, records.size)
        assertTrue(records.all { "success=false" in it && "IllegalStateException:not_ready" in it })
    }

    @Test
    fun `stopping prevents new requests and closing ignores outstanding callback`() {
        val f = Fixture()
        f.probe.start("flight", "CRUISE", null)
        f.probe.tick()
        f.probe.stop()
        f.reply(0)
        assertEquals("false", f.results.single()["scopeActive"])
        f.now = 200_000_000L
        f.probe.tick()
        assertEquals(1, f.callbacks.size)
        f.probe.start("next", "ACQUIRING", null)
        f.probe.tick()
        f.probe.close()
        f.reply(1)
        f.now = 400_000_000L
        f.probe.tick()
        assertEquals(2, f.callbacks.size)
        assertEquals(1, f.results.size)
        assertFalse(f.results.any { it["attemptId"] == "next" })
    }

    private class Fixture {
        var now = 0L
        val callbacks = mutableListOf<(VelocityReadDiagnostics.Velocity?, String?) -> Unit>()
        val results = mutableListOf<Map<String, String>>()
        val probe = VelocityReadDiagnostics(
            readVelocity = { callbacks += it },
            listenerSnapshot = { VelocityReadDiagnostics.ListenerSample(0.3, 0.0, 0.0, 0L) },
            record = { event, _, details ->
                if (event == "velocity_read_result") {
                    results += details.split(' ').associate { token ->
                        val pair = token.split('=', limit = 2)
                        pair[0] to pair[1]
                    }
                }
            },
            clock = { now },
        )

        fun reply(index: Int) = callbacks[index](VelocityReadDiagnostics.Velocity(0.3, 0.0, 0.0), null)
    }
}
