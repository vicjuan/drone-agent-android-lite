package com.durendal.droneagent.lite

import java.io.File
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executor
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class TapeCaptureRecorderTest {

    private val root: File = File.createTempFile("tape-recorder-test", "").let { file ->
        file.delete()
        file.mkdirs()
        file
    }
    private val logLines = mutableListOf<String>()
    private val directExecutor = Executor { it.run() }

    @After
    fun tearDown() {
        root.deleteRecursively()
    }

    @Test
    fun `a disarmed recorder writes nothing and keeps no window`() {
        val recorder = recorder()

        repeat(10) { recorder.offer(capture(it)) }
        recorder.trigger("loss")

        assertFalse(recorder.isArmed)
        assertEquals(0, recorder.savedCaptureCount)
        assertEquals(emptyList<String>(), savedNames())
    }

    @Test
    fun `a trigger writes the buffered run-up and the frames that follow`() {
        val recorder = recorder(leading = 2, trailing = 2)
        recorder.arm()

        repeat(3) { recorder.offer(capture(it)) }
        recorder.trigger("loss")
        recorder.offer(capture(3))
        recorder.offer(capture(4))

        assertEquals(
            listOf(
                "000000000-loss-before",
                "000000001-loss-before",
                "000000002-loss-at",
                "000000003-loss-after",
            ),
            savedNames(),
        )
    }

    @Test
    fun `the run-up keeps the newest frames when more arrive than fit`() {
        val recorder = recorder(leading = 2, trailing = 0)
        recorder.arm()

        repeat(5) { recorder.offer(capture(it)) }
        recorder.trigger("loss")

        val written = savedNames().map { TapeCaptureCodec.read(File(root, it)).metadata["frame"] }
        assertEquals(listOf("3", "4"), written)
    }

    @Test
    fun `frames offered after the trailing window are buffered again not written`() {
        val recorder = recorder(leading = 4, trailing = 1)
        recorder.arm()

        recorder.trigger("loss")
        recorder.offer(capture(0))
        recorder.offer(capture(1))
        recorder.offer(capture(2))

        assertEquals(listOf("000000000-loss-at"), savedNames())
    }

    @Test
    fun `a second trigger inside the trailing window extends the same event`() {
        val recorder = recorder(leading = 4, trailing = 2)
        recorder.arm()

        recorder.trigger("loss")
        recorder.offer(capture(0))
        recorder.trigger("loss")
        recorder.offer(capture(1))
        recorder.offer(capture(2))

        assertEquals(
            listOf(
                "000000000-loss-at",
                "000000001-loss-at",
                "000000002-loss-after",
            ),
            savedNames(),
        )
    }

    @Test
    fun `disarming drops the buffered window so a later event cannot mix flights`() {
        val recorder = recorder(leading = 4, trailing = 0)
        recorder.arm()
        repeat(3) { recorder.offer(capture(it)) }

        recorder.disarm()
        recorder.arm()
        recorder.offer(capture(9))
        recorder.trigger("loss")

        val written = savedNames().map { TapeCaptureCodec.read(File(root, it)).metadata["frame"] }
        assertEquals(listOf("9"), written)
    }

    @Test
    fun `a failing write is reported and never propagates into the image pipeline`() {
        val unwritable = File(root, "blocked").apply { writeText("not a directory") }
        val recorder = TapeCaptureRecorder(
            root = unwritable,
            log = logLines::add,
            leadingFrameCount = 1,
            trailingFrameCount = 0,
            store = TapeCaptureStore(unwritable),
            writer = directExecutor,
        )
        recorder.arm()

        recorder.offer(capture(0))
        recorder.trigger("loss")

        assertTrue(logLines.toString(), logLines.any { it.startsWith("tape capture write failed") })
    }

    @Test
    fun `a new recorder continues the numbering already on disk`() {
        val first = recorder(leading = 1, trailing = 0)
        first.arm()
        first.offer(capture(0))
        first.trigger("loss")

        val second = recorder(leading = 1, trailing = 0)
        second.arm()
        second.offer(capture(1))
        second.trigger("loss")

        assertEquals(listOf("000000000-loss-before", "000000001-loss-before"), savedNames())
    }

    @Test
    fun `a writer that cannot keep up drops the oldest and never grows without bound`() {
        val released = CountDownLatch(1)
        val writer = Executors.newSingleThreadExecutor()
        // Occupy the writer thread so the drain cannot start: this is the state a
        // slow filesystem puts the recorder in, and the state in which an
        // unbounded queue would accumulate whole frames until the app died.
        writer.execute { released.await(20, TimeUnit.SECONDS) }
        val recorder = TapeCaptureRecorder(
            root = root,
            log = { logLines.add(it) },
            leadingFrameCount = 32,
            trailingFrameCount = 0,
            maximumPendingWrites = 2,
            store = TapeCaptureStore(root),
            writer = writer,
        )
        recorder.arm()
        repeat(20) { recorder.offer(capture(it)) }

        recorder.trigger("loss")

        assertEquals(2, recorder.pendingCaptureCount)
        assertEquals(18, recorder.droppedCaptureCount)
        assertEquals(0, recorder.savedCaptureCount)

        released.countDown()
        writer.shutdown()
        assertTrue(writer.awaitTermination(20, TimeUnit.SECONDS))
        assertEquals(2, recorder.savedCaptureCount)
        assertEquals(0, recorder.pendingCaptureCount)
        // The survivors are the newest frames, which are the ones nearest the event.
        assertEquals(
            listOf("18", "19"),
            savedNames().map { TapeCaptureCodec.read(File(root, it)).metadata["frame"] },
        )
        assertTrue(logLines.toString(), logLines.any { it.contains("writer-behind") })
    }

    @Test
    fun `a store failure is counted as failed rather than saved`() {
        val unwritable = File(root, "blocked").apply { writeText("not a directory") }
        val recorder = TapeCaptureRecorder(
            root = unwritable,
            log = { logLines.add(it) },
            leadingFrameCount = 1,
            trailingFrameCount = 0,
            store = TapeCaptureStore(unwritable),
            writer = directExecutor,
        )
        recorder.arm()

        recorder.offer(capture(0))
        recorder.trigger("loss")

        assertEquals(0, recorder.savedCaptureCount)
        assertEquals(1, recorder.failedCaptureCount)
    }

    @Test
    fun `a capture being written still counts as pending`() {
        val entered = CountDownLatch(1)
        val release = CountDownLatch(1)
        val writer = Executors.newSingleThreadExecutor()
        val recorder = TapeCaptureRecorder(
            root = root,
            log = { logLines.add(it) },
            leadingFrameCount = 8,
            trailingFrameCount = 0,
            maximumPendingWrites = 8,
            store = BlockingSink(TapeCaptureStore(root), entered, release),
            writer = writer,
        )
        recorder.arm()
        repeat(3) { recorder.offer(capture(it)) }

        recorder.trigger("loss")
        assertTrue("writer never started", entered.await(20, TimeUnit.SECONDS))

        // One capture is inside the sink and two are queued behind it. Counting
        // only the queue here would report 2 and lose the one being written.
        assertEquals(3, recorder.pendingCaptureCount)
        assertEquals(0, recorder.savedCaptureCount)

        release.countDown()
        writer.shutdown()
        assertTrue(writer.awaitTermination(20, TimeUnit.SECONDS))
        assertEquals(0, recorder.pendingCaptureCount)
        assertEquals(3, recorder.savedCaptureCount)
    }

    /** Holds the first save open so the in-flight capture is observable. */
    private class BlockingSink(
        private val delegate: TapeCaptureSink,
        private val entered: CountDownLatch,
        private val release: CountDownLatch,
    ) : TapeCaptureSink by delegate {
        private var first = true

        override fun save(capture: TapeCapture, sequence: Long, reason: String): File {
            if (first) {
                first = false
                entered.countDown()
                release.await(20, TimeUnit.SECONDS)
            }
            return delegate.save(capture, sequence, reason)
        }
    }

    private fun recorder(leading: Int = 4, trailing: Int = 4) = TapeCaptureRecorder(
        root = root,
        log = logLines::add,
        leadingFrameCount = leading,
        trailingFrameCount = trailing,
        store = TapeCaptureStore(root),
        writer = directExecutor,
    )

    private fun savedNames(): List<String> =
        root.listFiles { file -> file.isDirectory && File(file, TapeCaptureCodec.MANIFEST_NAME).isFile }
            ?.map { it.name }
            ?.sorted()
            ?: emptyList()

    private fun capture(frame: Int) = TapeCapture(
        metadata = linkedMapOf("frame" to frame.toString()),
        frame = TapeCapturePlane(
            name = TapeCapture.FRAME_PLANE_NAME,
            width = 2,
            height = 2,
            channels = 4,
            pixels = ByteArray(16) { frame.toByte() },
        ),
    )
}
