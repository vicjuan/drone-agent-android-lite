package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Test

class TapeLuminancePolicyTest {
    @Test
    fun `bright background cannot raise black threshold into cardboard range`() {
        assertEquals(105.0, TapeLuminancePolicy.effectiveThreshold(168.0), 0.0)
    }

    @Test
    fun `dim scene keeps its lower adaptive threshold`() {
        assertEquals(62.0, TapeLuminancePolicy.effectiveThreshold(62.0), 0.0)
    }
}
