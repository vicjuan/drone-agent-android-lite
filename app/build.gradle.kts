import java.util.Properties

plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

val djiMsdkVersion = "5.18.0"

// The DJI app key is a secret and is NEVER committed. It is read from
// dji-app-key.properties in the repo root (gitignored) and injected into the
// merged manifest as meta-data com.dji.sdk.API_KEY, which is the only place the
// MSDK reads it from.
val djiApiKey: String = rootProject.file("dji-app-key.properties").let { file ->
    if (!file.exists()) {
        ""
    } else {
        Properties().apply { file.inputStream().use(::load) }
            .getProperty("DJI_API_KEY")
            .orEmpty()
            .trim()
    }
}

android {
    namespace = "com.durendal.droneagent.lite"
    compileSdk = 34

    defaultConfig {
        // A DJI app key is bound to ONE package name. This value reuses the key
        // already provisioned for the main drone-agent-android app. Changing it
        // requires issuing a new app key in the DJI Developer Center, otherwise
        // registration fails with an invalid-app-key error.
        applicationId = "com.durendal.droneagent.app"
        minSdk = 24
        targetSdk = 34
        versionCode = 1
        versionName = "0.1.0-lite"

        manifestPlaceholders["djiApiKey"] = djiApiKey

        // MSDK ships arm64 natives only for the aircraft package we use; keeping
        // one ABI also keeps the APK small and the build fast.
        ndk { abiFilters += "arm64-v8a" }
    }

    buildTypes {
        release { isMinifyEnabled = false }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions {
        jvmTarget = "17"
        // MSDK 5.18.0 ships Kotlin metadata newer than the compiler in use.
        freeCompilerArgs += "-Xskip-metadata-version-check"
    }

    packaging {
        jniLibs {
            // MSDK loads its natives with System.loadLibrary from the APK layout.
            useLegacyPackaging = true
            pickFirsts += listOf("lib/arm64-v8a/libc++_shared.so")
        }
    }
}

dependencies {
    implementation("com.dji:dji-sdk-v5-aircraft:$djiMsdkVersion")
    // MSDK's analytics engine touches androidx.core.app.ActivityCompat during
    // SDKManager.init and the aar does not declare the dependency itself, so
    // init crashes with NoClassDefFoundError before INITIALIZE_COMPLETE.
    // Version matches the main drone-agent-android app.
    implementation("androidx.core:core:1.13.1")
    // "-provided" holds the API stubs only; it MUST stay compileOnly or the
    // runtime classes get shadowed and registration crashes.
    compileOnly("com.dji:dji-sdk-v5-aircraft-provided:$djiMsdkVersion")
    // Registration talks to DJI servers through this implementation.
    runtimeOnly("com.dji:dji-sdk-v5-networkImp:$djiMsdkVersion")
    testImplementation("junit:junit:4.13.2")
}
