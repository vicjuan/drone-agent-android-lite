package com.durendal.droneagent.lite

import java.io.File
import java.io.IOException
import org.junit.After
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Assert.fail
import org.junit.Test

class TapeCaptureTest {

    private val root: File = createTempDirectory()

    @After
    fun tearDown() {
        root.deleteRecursively()
    }

    @Test
    fun `a written capture reads back byte identical`() {
        val capture = capture(
            frameWidth = 7,
            frameHeight = 5,
            metadata = linkedMapOf(
                "frame.timestampNanos" to "123456789",
                "gimbal.pitchDegrees" to "-90.0",
                "detection" to "none",
            ),
            maskNames = listOf("blackMask", "floorMask"),
        )

        TapeCaptureCodec.write(capture, File(root, "one"))
        val restored = TapeCaptureCodec.read(File(root, "one"))

        assertEquals(capture.metadata, restored.metadata)
        assertEquals(capture.frame.width, restored.frame.width)
        assertEquals(capture.frame.height, restored.frame.height)
        assertEquals(capture.frame.channels, restored.frame.channels)
        assertArrayEquals(capture.frame.pixels, restored.frame.pixels)
        assertEquals(
            capture.masks.map { it.name },
            restored.masks.map { it.name },
        )
        capture.masks.zip(restored.masks).forEach { (original, read) ->
            assertEquals(original.name, original.width, read.width)
            assertEquals(original.name, original.height, read.height)
            assertEquals(original.name, original.channels, read.channels)
            assertArrayEquals(original.pixels, read.pixels)
        }
    }

    @Test
    fun `plane geometry never leaks into replayed metadata`() {
        val capture = capture(metadata = linkedMapOf("detection" to "none"), maskNames = listOf("blackMask"))

        TapeCaptureCodec.write(capture, File(root, "one"))
        val restored = TapeCaptureCodec.read(File(root, "one"))

        assertEquals(setOf("detection"), restored.metadata.keys)
    }

    @Test
    fun `a truncated manifest fails loudly instead of replaying a partial capture`() {
        val directory = File(root, "one")
        TapeCaptureCodec.write(capture(maskNames = listOf("blackMask")), directory)
        File(directory, TapeCaptureCodec.MANIFEST_NAME).writeText("planes=frame,blackMask\n")

        try {
            TapeCaptureCodec.read(directory)
            fail("expected a missing-geometry failure")
        } catch (error: IOException) {
            assertTrue(error.message.orEmpty(), error.message.orEmpty().contains("geometry"))
        }
    }

    @Test
    fun `a missing manifest is reported rather than treated as an empty capture`() {
        val directory = File(root, "empty").apply { mkdirs() }

        try {
            TapeCaptureCodec.read(directory)
            fail("expected a missing-manifest failure")
        } catch (error: IOException) {
            assertTrue(error.message.orEmpty(), error.message.orEmpty().contains(TapeCaptureCodec.MANIFEST_NAME))
        }
    }

    @Test
    fun `the store keeps the newest captures within the count limit`() {
        val store = TapeCaptureStore(root, maximumCaptures = 3, maximumTotalBytes = Long.MAX_VALUE)

        repeat(5) { index -> store.save(capture(), sequence = index.toLong(), reason = "loss") }

        val remaining = store.captures().map { it.name }
        assertEquals(3, remaining.size)
        assertEquals(listOf("000000002-loss", "000000003-loss", "000000004-loss"), remaining)
    }

    @Test
    fun `the store evicts by size and always keeps the capture just written`() {
        val store = TapeCaptureStore(root, maximumCaptures = 100, maximumTotalBytes = 1L)

        store.save(capture(), sequence = 0, reason = "loss")
        store.save(capture(), sequence = 1, reason = "loss")

        val remaining = store.captures().map { it.name }
        assertEquals(listOf("000000001-loss"), remaining)
        assertNotNull(TapeCaptureCodec.read(File(root, "000000001-loss")))
    }

    @Test
    fun `a reason that is not a safe file name is sanitised`() {
        val store = TapeCaptureStore(root)

        val directory = store.save(capture(), sequence = 12, reason = "path lost / glare")

        assertEquals("000000012-path-lost---glare", directory.name)
    }

    @Test
    fun `an empty reason still produces a readable directory name`() {
        val store = TapeCaptureStore(root)

        val directory = store.save(capture(), sequence = 3, reason = "")

        assertEquals("000000003-unspecified", directory.name)
    }

    @Test
    fun `a directory without a manifest is not reported as a capture`() {
        val store = TapeCaptureStore(root)
        store.save(capture(), sequence = 0, reason = "loss")
        File(root, "scratch").mkdirs()

        assertEquals(listOf("000000000-loss"), store.captures().map { it.name })
    }

    @Test
    fun `total bytes counts every written capture`() {
        val store = TapeCaptureStore(root, maximumCaptures = 10, maximumTotalBytes = Long.MAX_VALUE)
        store.save(capture(), sequence = 0, reason = "loss")
        val single = store.totalBytes()
        store.save(capture(), sequence = 1, reason = "loss")

        assertTrue("single=$single total=${store.totalBytes()}", store.totalBytes() > single)
    }

    @Test
    fun `a capture becomes visible only once it is complete`() {
        val store = TapeCaptureStore(root)
        val big = capture(frameWidth = 64, frameHeight = 64, maskNames = listOf("blackMask"))

        val directory = store.save(big, sequence = 0, reason = "loss")

        assertEquals(listOf(directory.name), store.captures().map { it.name })
        assertEquals(
            "no scratch directory may survive a completed write",
            emptyList<String>(),
            root.listFiles()?.map { it.name }?.filter { it != directory.name }.orEmpty(),
        )
    }

    @Test
    fun `a scratch directory from a killed run is reclaimed`() {
        val scratch = File(root, "incomplete-000000007-loss").apply { mkdirs() }
        File(scratch, "frame.raw.gz").writeBytes(ByteArray(4096))

        val store = TapeCaptureStore(root)

        assertFalse(scratch.exists())
        assertEquals(emptyList<String>(), store.captures().map { it.name })
    }

    @Test
    fun `cleanup never touches a directory the store did not create`() {
        // A cleanup that deletes every manifest-less directory would take out
        // anything a caller keeps beside the captures. The store may only
        // reclaim names it could have written itself.
        val foreign = File(root, "flight-videos").apply { mkdirs() }
        File(foreign, "clip.mp4").writeBytes(ByteArray(16))
        val notOurScratch = File(root, "incomplete-notes").apply { mkdirs() }
        val ourScratch = File(root, "incomplete-000000001-loss").apply { mkdirs() }

        TapeCaptureStore(root)

        assertTrue("an unrelated directory was deleted", foreign.isDirectory)
        assertTrue(File(foreign, "clip.mp4").isFile)
        assertTrue("a foreign incomplete- name was deleted", notOurScratch.isDirectory)
        assertFalse("the store's own scratch survived", ourScratch.exists())
    }

    @Test
    fun `a failed write leaves nothing behind and does not block the next capture`() {
        val store = TapeCaptureStore(root)
        store.save(capture(), sequence = 0, reason = "loss")
        // A root the process cannot write into is the controllable stand-in for
        // a full or failing filesystem: the scratch directory cannot be created,
        // so the write fails at the first step.
        assertTrue("could not make the store root read-only", root.setWritable(false))
        assertFalse(
            "the environment did not enforce the read-only root",
            File(root, "probe").mkdirs(),
        )

        assertThrows(IOException::class.java) {
            store.save(capture(), sequence = 1, reason = "loss")
        }

        assertTrue(root.setWritable(true))
        assertEquals(
            "a failed write must leave no scratch directory",
            emptyList<String>(),
            root.listFiles()?.map { it.name }?.filter { it.startsWith("incomplete-") }.orEmpty(),
        )
        assertEquals(listOf("000000000-loss"), store.captures().map { it.name })

        val recovered = store.save(capture(), sequence = 2, reason = "loss")

        assertTrue(recovered.isDirectory)
        assertEquals(
            listOf("000000000-loss", "000000002-loss"),
            store.captures().map { it.name },
        )
        assertNotNull(TapeCaptureCodec.read(recovered))
    }

    @Test
    fun `a write that fails while publishing removes the scratch it built`() {
        val store = TapeCaptureStore(root)
        // An existing capture directory that cannot be replaced. The scratch
        // directory is written in full and only the final publish fails, so this
        // is the case where there really is a half-built capture to clean up.
        val occupied = File(root, "000000000-loss").apply { mkdirs() }
        File(occupied, TapeCaptureCodec.MANIFEST_NAME).writeText("planes=\n")
        assertTrue("could not make the destination unreplaceable", occupied.setWritable(false))

        assertThrows(IOException::class.java) {
            store.save(
                capture(maskNames = listOf("blackMask")),
                sequence = 0,
                reason = "loss",
            )
        }

        assertTrue(occupied.setWritable(true))
        assertEquals(
            "a failed publish must leave no scratch directory",
            emptyList<String>(),
            root.listFiles()?.map { it.name }?.filter { it.startsWith("incomplete-") }.orEmpty(),
        )
        // The occupant is untouched: a failed write must not damage what is there.
        assertEquals("planes=\n", File(occupied, TapeCaptureCodec.MANIFEST_NAME).readText())
    }

    private fun capture(
        frameWidth: Int = 4,
        frameHeight: Int = 3,
        metadata: Map<String, String> = emptyMap(),
        maskNames: List<String> = emptyList(),
    ): TapeCapture = TapeCapture(
        metadata = metadata,
        frame = TapeCapturePlane(
            name = TapeCapture.FRAME_PLANE_NAME,
            width = frameWidth,
            height = frameHeight,
            channels = 4,
            pixels = ByteArray(frameWidth * frameHeight * 4) { (it * 7 % 251).toByte() },
        ),
        masks = maskNames.mapIndexed { maskIndex, name ->
            TapeCapturePlane(
                name = name,
                width = frameWidth,
                height = frameHeight,
                channels = 1,
                pixels = ByteArray(frameWidth * frameHeight) { ((it + maskIndex) % 2 * 255).toByte() },
            )
        },
    )

    private fun createTempDirectory(): File =
        File.createTempFile("tape-capture-test", "").let { file ->
            file.delete()
            file.mkdirs()
            file
        }
}
