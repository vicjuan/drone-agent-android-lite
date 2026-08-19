package com.durendal.droneagent.lite

import java.io.File
import java.io.IOException
import java.util.zip.GZIPInputStream
import java.util.zip.GZIPOutputStream

/**
 * One single-channel or RGBA image belonging to a capture.
 *
 * [pixels] is stored verbatim, so a decoded plane is byte-identical to the one
 * the detector held. Equality is deliberately not defined: callers compare
 * pixels explicitly rather than relying on array identity.
 */
internal class TapeCapturePlane(
    val name: String,
    val width: Int,
    val height: Int,
    val channels: Int,
    val pixels: ByteArray,
) {
    init {
        require(name.isNotEmpty() && name.all { it.isLetterOrDigit() || it == '-' }) {
            "plane name must be a non-empty [A-Za-z0-9-] token, was '$name'"
        }
        require(width > 0 && height > 0) { "plane dimensions must be positive" }
        require(channels > 0) { "channels must be positive" }
        require(pixels.size.toLong() == width.toLong() * height * channels) {
            "plane '$name' holds ${pixels.size} bytes, expected $width x $height x $channels"
        }
    }
}

/**
 * A replayable detector input: the exact frame bytes the detector consumed, the
 * intermediate masks that frame produced, and the verdict it reached.
 *
 * The whole point of the format is that nothing here is a screen recording. A
 * capture can be fed back through the pipeline offline, so a failure can be
 * re-examined after the segmentation or geometry code changes rather than
 * argued about from an overlay that no longer reflects the current build.
 *
 * [metadata] is free-form and ordered: it carries frame timestamp, gimbal pose,
 * height, detection mode, rejection counts and the verdict. Keeping it as text
 * rather than a fixed class means a new diagnostic never invalidates captures
 * recorded by an older build.
 */
internal class TapeCapture(
    val metadata: Map<String, String>,
    val frame: TapeCapturePlane,
    val masks: List<TapeCapturePlane> = emptyList(),
) {
    init {
        require(frame.name == FRAME_PLANE_NAME) {
            "the frame plane must be named '$FRAME_PLANE_NAME', was '${frame.name}'"
        }
        require(masks.none { it.name == FRAME_PLANE_NAME }) {
            "a mask may not reuse the reserved name '$FRAME_PLANE_NAME'"
        }
        require(masks.map { it.name }.toSet().size == masks.size) {
            "mask names must be unique"
        }
        metadata.keys.forEach { key ->
            require(key.isNotEmpty() && key.none { it == '=' || it == '\n' }) {
                "metadata key '$key' must be non-empty and free of '=' and newlines"
            }
        }
        metadata.values.forEach { value ->
            require(value.none { it == '\n' }) { "metadata values must be single-line" }
        }
    }

    internal companion object {
        const val FRAME_PLANE_NAME = "frame"
    }
}

/**
 * Reads and writes a capture as a plain directory.
 *
 * The format is deliberately dependency-free — gzipped raw planes plus a
 * `key=value` manifest — so the same code runs in a JVM unit test and on the
 * aircraft, and so a capture recorded today stays readable without the build
 * that produced it. Human-viewable exports are a separate concern; this is the
 * canonical, lossless record.
 */
internal object TapeCaptureCodec {

    fun write(capture: TapeCapture, directory: File) {
        if (!directory.exists() && !directory.mkdirs()) {
            throw IOException("cannot create capture directory ${directory.absolutePath}")
        }
        writePlane(capture.frame, directory)
        capture.masks.forEach { writePlane(it, directory) }
        File(directory, MANIFEST_NAME).writeText(manifestText(capture))
    }

    fun read(directory: File): TapeCapture {
        val manifest = File(directory, MANIFEST_NAME)
        if (!manifest.isFile) {
            throw IOException("no $MANIFEST_NAME in ${directory.absolutePath}")
        }
        val entries = LinkedHashMap<String, String>()
        manifest.forEachLine { line ->
            if (line.isEmpty()) return@forEachLine
            val separator = line.indexOf('=')
            if (separator <= 0) throw IOException("malformed manifest line '$line'")
            entries[line.substring(0, separator)] = line.substring(separator + 1)
        }
        val planeNames = entries.remove(PLANES_KEY)?.split(',')?.filter { it.isNotEmpty() }
            ?: throw IOException("manifest has no $PLANES_KEY entry")
        val planes = planeNames.map { name -> readPlane(name, entries, directory) }
        val frame = planes.firstOrNull { it.name == TapeCapture.FRAME_PLANE_NAME }
            ?: throw IOException("capture has no '${TapeCapture.FRAME_PLANE_NAME}' plane")
        planeNames.forEach { name ->
            entries.remove(geometryKey(name))
        }
        return TapeCapture(
            metadata = entries,
            frame = frame,
            masks = planes.filter { it !== frame },
        )
    }

    private fun writePlane(plane: TapeCapturePlane, directory: File) {
        File(directory, planeFileName(plane.name)).outputStream().use { fileStream ->
            GZIPOutputStream(fileStream).use { it.write(plane.pixels) }
        }
    }

    private fun readPlane(
        name: String,
        entries: Map<String, String>,
        directory: File,
    ): TapeCapturePlane {
        val geometry = entries[geometryKey(name)]
            ?: throw IOException("manifest has no geometry for plane '$name'")
        val parts = geometry.split('x')
        if (parts.size != 3) throw IOException("malformed geometry '$geometry' for plane '$name'")
        val width = parts[0].toIntOrNull()
        val height = parts[1].toIntOrNull()
        val channels = parts[2].toIntOrNull()
        if (width == null || height == null || channels == null) {
            throw IOException("non-numeric geometry '$geometry' for plane '$name'")
        }
        val file = File(directory, planeFileName(name))
        if (!file.isFile) throw IOException("missing plane file ${file.absolutePath}")
        val pixels = file.inputStream().use { fileStream ->
            GZIPInputStream(fileStream).use { it.readBytes() }
        }
        return TapeCapturePlane(name, width, height, channels, pixels)
    }

    private fun manifestText(capture: TapeCapture): String {
        val planes = listOf(capture.frame) + capture.masks
        return buildString {
            append(PLANES_KEY).append('=').append(planes.joinToString(",") { it.name }).append('\n')
            planes.forEach { plane ->
                append(geometryKey(plane.name)).append('=')
                append(plane.width).append('x').append(plane.height).append('x')
                append(plane.channels).append('\n')
            }
            capture.metadata.forEach { (key, value) ->
                append(key).append('=').append(value).append('\n')
            }
        }
    }

    private fun planeFileName(name: String): String = "$name.raw.gz"

    private fun geometryKey(name: String): String = "plane.$name.geometry"

    const val MANIFEST_NAME = "capture.txt"
    private const val PLANES_KEY = "planes"
}

/**
 * A size-bounded directory of captures, newest kept.
 *
 * Evidence is worthless if collecting it fills the aircraft's storage mid-flight,
 * so the store enforces both a count and a byte ceiling and drops the oldest
 * captures first. Eviction happens after the write, so the newest capture — the
 * one an operator just triggered — is never the one discarded.
 *
 * Writes are atomic: a capture is assembled in a scratch directory and only
 * becomes visible by rename once every plane and the manifest are on disk. A
 * process killed mid-write therefore leaves no half-capture that replay would
 * read as real, and no files that the size ceiling cannot see. Whatever a
 * previous run did leave behind is purged when a store is constructed.
 */
internal class TapeCaptureStore(
    private val root: File,
    private val maximumCaptures: Int = DEFAULT_MAXIMUM_CAPTURES,
    private val maximumTotalBytes: Long = DEFAULT_MAXIMUM_TOTAL_BYTES,
) {
    init {
        require(maximumCaptures > 0) { "maximumCaptures must be > 0" }
        require(maximumTotalBytes > 0L) { "maximumTotalBytes must be > 0" }
        purgeIncomplete()
    }

    /**
     * Writes [capture] under a directory named by [sequence] and [reason], then
     * evicts until the store is inside both limits. Returns the directory.
     */
    fun save(capture: TapeCapture, sequence: Long, reason: String): File {
        val directory = File(root, captureDirectoryName(sequence, reason))
        val scratch = File(root, "$INCOMPLETE_PREFIX${directory.name}")
        scratch.deleteRecursively()
        try {
            TapeCaptureCodec.write(capture, scratch)
            directory.deleteRecursively()
            if (!scratch.renameTo(directory)) {
                throw IOException("cannot publish capture ${directory.absolutePath}")
            }
        } catch (error: Throwable) {
            scratch.deleteRecursively()
            throw error
        }
        evictUntilWithinLimits()
        return directory
    }

    /** Capture directories, oldest first by sequence. */
    fun captures(): List<File> = completeDirectories().sortedBy { it.name }

    fun totalBytes(): Long = captures().sumOf { directorySize(it) }

    /**
     * One past the highest sequence already on disk, so a new recording session
     * appends to the store instead of overwriting the previous session's
     * directories. Unparsable names are ignored rather than resetting the count.
     */
    fun nextSequence(): Long =
        (captures().mapNotNull { it.name.substringBefore('-').toLongOrNull() }.maxOrNull() ?: -1L) + 1L

    /**
     * Removes anything under the root that is not a complete capture: scratch
     * directories from a killed write, and directories left by an older build
     * that published planes before their manifest. Either kind is invisible to
     * [captures] and would otherwise occupy storage no ceiling accounts for.
     */
    private fun purgeIncomplete() {
        root.listFiles()
            ?.filter { it.isDirectory && !File(it, TapeCaptureCodec.MANIFEST_NAME).isFile }
            ?.forEach { it.deleteRecursively() }
    }

    private fun completeDirectories(): List<File> =
        root.listFiles { file -> file.isDirectory && File(file, TapeCaptureCodec.MANIFEST_NAME).isFile }
            ?.toList()
            ?: emptyList()

    private fun evictUntilWithinLimits() {
        val remaining = captures().toMutableList()
        val sizes = remaining.associateWith(::directorySize).toMutableMap()
        while (
            remaining.size > maximumCaptures ||
            (remaining.size > 1 && sizes.values.sum() > maximumTotalBytes)
        ) {
            val oldest = remaining.removeAt(0)
            sizes.remove(oldest)
            oldest.deleteRecursively()
        }
    }

    private fun directorySize(directory: File): Long =
        directory.listFiles()?.sumOf { if (it.isFile) it.length() else directorySize(it) } ?: 0L

    /**
     * Zero-padded so lexicographic directory order is capture order, which is
     * what makes oldest-first eviction and offline replay ordering trivial.
     */
    private fun captureDirectoryName(sequence: Long, reason: String): String {
        val safeReason = reason.map { if (it.isLetterOrDigit() || it == '-') it else '-' }
            .joinToString("")
            .take(MAXIMUM_REASON_LENGTH)
            .ifEmpty { "unspecified" }
        return "%09d-%s".format(sequence, safeReason)
    }

    internal companion object {
        const val DEFAULT_MAXIMUM_CAPTURES = 40
        const val DEFAULT_MAXIMUM_TOTAL_BYTES = 256L * 1024 * 1024
        private const val MAXIMUM_REASON_LENGTH = 40
        private const val INCOMPLETE_PREFIX = "incomplete-"
    }
}
