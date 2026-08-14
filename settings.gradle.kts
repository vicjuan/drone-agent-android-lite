pluginManagement {
    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}

dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.PREFER_SETTINGS)
    repositories {
        google()
        // DJI MSDK V5 artifacts (com.dji:dji-sdk-v5-*) live on Maven Central.
        mavenCentral()
    }
}

rootProject.name = "drone-agent-android-lite"

include(":app")
