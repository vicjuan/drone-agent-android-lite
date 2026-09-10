package com.durendal.droneagent.lite

import java.io.File
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class FlightProfilerTest {

    @Test
    fun `records monotonic timing rows and sanitizes details`() {
        val file = File.createTempFile("flight-profile", ".tsv")
        var nowNanos = 1_000_000_000L
        val profiler = FlightProfiler(file) { nowNanos }

        nowNanos = 1_125_000_000L
        profiler.record(
            event = "vision",
            atNanos = nowNanos,
            frameNanos = 1_050_000_000L,
            durationNanos = 25_000_000L,
            details = "detected=true\tmode=PATH\n",
        )
        profiler.close()

        val lines = file.readLines()
        assertEquals(
            "elapsedMs\tmonotonicNanos\tevent\tframeNanos\tdurationMs\tdetails",
            lines.first(),
        )
        assertEquals(
            "125\t1125000000\tvision\t1050000000\t25\tdetected=true mode=PATH ",
            lines.single { it.contains("\tvision\t") },
        )
        assertTrue(file.delete())
    }

    @Test
    fun `lazy record builds details before close returns`() {
        val file = File.createTempFile("flight-profile-lazy", ".tsv")
        val profiler = FlightProfiler(file) { 1_000_000_000L }
        var detailsBuilt = false

        profiler.recordLazy(event = "vision") {
            detailsBuilt = true
            "sequence=42"
        }
        profiler.close()

        assertTrue(detailsBuilt)
        assertTrue(file.readText().contains("\tvision\t0\t0\tsequence=42"))
        assertTrue(file.delete())
    }

    @Test
    fun `lazy record after close does not build details`() {
        val file = File.createTempFile("flight-profile-closed", ".tsv")
        val profiler = FlightProfiler(file) { 1_000_000_000L }
        var detailsBuilt = false
        profiler.close()

        profiler.recordLazy(event = "vision") {
            detailsBuilt = true
            "must-not-be-written"
        }

        assertEquals(false, detailsBuilt)
        assertEquals(1, file.readLines().size)
        assertTrue(file.delete())
    }

    @Test
    fun `checkpoint publishes preceding records without closing the trace`() {
        val file = File.createTempFile("flight-profile-checkpoint", ".tsv")
        val profiler = FlightProfiler(file)
        val flushed = CountDownLatch(1)
        var failure: Throwable? = null
        try {
            profiler.recordLazy(event = "straight_test_stop") { "attemptId=round-1" }
            profiler.checkpoint {
                failure = it
                flushed.countDown()
            }
            assertTrue(flushed.await(2, TimeUnit.SECONDS))
            assertEquals(null, failure)
            assertTrue(file.readText().contains("\tstraight_test_stop\t0\t0\tattemptId=round-1\n"))
            profiler.record(event = "velocity", details = "x=0.0 y=0.0")
            profiler.close()
            assertTrue(file.readText().contains("\tvelocity\t0\t0\tx=0.0 y=0.0\n"))
        } finally {
            profiler.close()
            file.delete()
        }
    }

    @Test
    fun `checkpoint after close reports failure instead of claiming durable data`() {
        val file = File.createTempFile("flight-profile-checkpoint-closed", ".tsv")
        val profiler = FlightProfiler(file)
        profiler.close()
        var failure: Throwable? = null
        profiler.checkpoint { failure = it }
        assertTrue(failure is IllegalStateException)
        file.delete()
    }

    @Test
    fun `profile details retain field names for offline analysis`() {
        assertEquals(
            "forward=0.5 phase=TRACKING missing=null",
            profileDetails("forward" to 0.5, "phase" to "TRACKING", "missing" to null),
        )
    }
}
