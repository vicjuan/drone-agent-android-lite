package com.durendal.droneagent.lite

import java.io.File
import java.util.concurrent.Executor
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
        assertEquals(0, recorder.capturedFrameCount)
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
