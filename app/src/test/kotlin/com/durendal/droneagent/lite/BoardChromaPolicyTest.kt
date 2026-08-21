package com.durendal.droneagent.lite

import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class BoardChromaPolicyTest {
    @Test
    fun `tracked cardboard survives absolute chroma drift when its reference still matches`() {
        assertTrue(
            acceptsCorrugatedBoard(
                absoluteChromaMatch = false,
                overlapsPreviousRoute = true,
                matchesAcceptedBoardReference = true,
            ),
        )
    }

    @Test
    fun `a new route still requires absolute cardboard chroma`() {
        assertFalse(
            acceptsCorrugatedBoard(
                absoluteChromaMatch = false,
                overlapsPreviousRoute = false,
                matchesAcceptedBoardReference = true,
            ),
        )
    }

    @Test
    fun `an overlapping route cannot bypass both colour gates`() {
        assertFalse(
            acceptsCorrugatedBoard(
                absoluteChromaMatch = false,
                overlapsPreviousRoute = true,
                matchesAcceptedBoardReference = false,
            ),
        )
    }
}
