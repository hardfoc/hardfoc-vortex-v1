#include "WS2812TestThread.h"
#include "ConsolePort.h"
#include "hf_gpio_config.hpp"

static const char* TAG = "WS2812Test";

// Define the effect list and count
const WS2812Animator::Effect WS2812TestThread::s_effectList[] = {
    WS2812Animator::Effect::SolidColor,
    WS2812Animator::Effect::Rainbow,
    WS2812Animator::Effect::Breath,
    WS2812Animator::Effect::Chase,
    WS2812Animator::Effect::Blink,
    WS2812Animator::Effect::Larson
};

const size_t WS2812TestThread::s_effectCount = sizeof(s_effectList) / sizeof(s_effectList[0]);

WS2812TestThread::WS2812TestThread(uint8_t ledCount, uint32_t updateIntervalMs)
    : BaseThread("WS2812Test")
    , m_strip(nullptr)
    , m_animator(nullptr)
    , m_ledCount(ledCount)
    , m_updateIntervalMs(updateIntervalMs)
    , m_brightness(128)  // 50% brightness by default
    , m_currentEffect(WS2812Animator::Effect::SolidColor)
    , m_effectStartTime(0)
    , m_effectDuration(5000)  // 5 seconds per effect
    , m_effectIndex(0)
{
    console_info(TAG, "WS2812TestThread created with %d LEDs", ledCount);
}

WS2812TestThread::~WS2812TestThread()
{
    if (m_animator) {
        delete m_animator;
        m_animator = nullptr;
    }
    
    if (m_strip) {
        delete m_strip;
        m_strip = nullptr;
    }
    
    console_info(TAG, "WS2812TestThread destroyed");
}

bool WS2812TestThread::Initialize()
{
    console_info(TAG, "Initializing WS2812TestThread...");
    
    // Initialize pin configuration
    init_mcu_pinconfig();
    
    console_info(TAG, "WS2812TestThread initialized successfully");
    return true;
}

bool WS2812TestThread::Setup()
{
    console_info(TAG, "Setting up WS2812 hardware...");
    
    // Create WS2812 strip - use the pin from hf_gpio_config.hpp
    m_strip = new WS2812Strip(WS2812_LED_PIN, 0, m_ledCount);
    
    if (!m_strip) {
        console_error(TAG, "Failed to create WS2812Strip");
        return false;
    }
    
    // Initialize the strip
    esp_err_t ret = m_strip->begin();
    if (ret != ESP_OK) {
        console_error(TAG, "Failed to initialize WS2812 strip: error code %d", ret);
        delete m_strip;
        m_strip = nullptr;
        return false;
    }
    
    // Create animator
    m_animator = new WS2812Animator(*m_strip, m_ledCount);
    
    if (!m_animator) {
        console_error(TAG, "Failed to create WS2812Animator");
        delete m_strip;
        m_strip = nullptr;
        return false;
    }
    
    // Set initial brightness and effect
    m_strip->setBrightness(m_brightness);
    
    // Initialize timing
    m_effectStartTime = os_get_elapsed_time_msec();
    
    console_info(TAG, "WS2812 hardware setup complete");
    return true;
}

uint32_t WS2812TestThread::Step()
{
    // Check if it's time to change effects
    if (isTimeToChangeEffect()) {
        cycleToNextEffect();
    }
    
    // Run the current effect
    runCurrentEffect();
    
    return m_updateIntervalMs;
}

bool WS2812TestThread::Cleanup()
{
    console_info(TAG, "Cleaning up WS2812TestThread...");
    
    // Turn off all LEDs
    if (m_strip) {
        for (uint32_t i = 0; i < m_ledCount; i++) {
            m_strip->setPixel(i, 0);
        }
        m_strip->show();
    }
    
    // Clean up resources
    if (m_animator) {
        delete m_animator;
        m_animator = nullptr;
    }
    
    if (m_strip) {
        delete m_strip;
        m_strip = nullptr;
    }
    
    console_info(TAG, "WS2812TestThread cleanup complete");
    return true;
}

bool WS2812TestThread::ResetVariables()
{
    // Reset effect state
    m_currentEffect = WS2812Animator::Effect::SolidColor;
    m_effectIndex = 0;
    m_effectStartTime = os_get_elapsed_time_msec();
    
    console_info(TAG, "WS2812TestThread variables reset");
    return true;
}

void WS2812TestThread::SetBrightness(uint8_t brightness)
{
    m_brightness = brightness;
    if (m_strip) {
        m_strip->setBrightness(brightness);
    }
}

void WS2812TestThread::SetEffectDuration(uint32_t duration)
{
    m_effectDuration = duration;
}

void WS2812TestThread::cycleToNextEffect()
{
    m_effectIndex = (m_effectIndex + 1) % s_effectCount;
    m_currentEffect = s_effectList[m_effectIndex];
    m_effectStartTime = os_get_elapsed_time_msec();
    
    console_info(TAG, "Switched to effect: %s", getEffectName(m_currentEffect));
}

void WS2812TestThread::runCurrentEffect()
{
    if (!m_animator) {
        return;
    }
    
    // Set the current effect and run animation
    switch (m_currentEffect) {
        case WS2812Animator::Effect::SolidColor:
            {
                // Cycle through colors over time
                uint32_t elapsed = os_get_elapsed_time_msec() - m_effectStartTime;
                uint32_t colors[] = {0xFF0000, 0x00FF00, 0x0000FF, 0xFFFF00, 0xFF00FF, 0x00FFFF};
                uint32_t colorIndex = (elapsed / 1000) % (sizeof(colors) / sizeof(colors[0]));
                m_animator->setEffect(WS2812Animator::Effect::SolidColor, colors[colorIndex]);
            }
            break;
        case WS2812Animator::Effect::Rainbow:
            m_animator->setEffect(WS2812Animator::Effect::Rainbow);
            break;
        case WS2812Animator::Effect::Breath:
            m_animator->setEffect(WS2812Animator::Effect::Breath, 0x0000FF);  // Blue breathing
            break;
        case WS2812Animator::Effect::Chase:
            m_animator->setEffect(WS2812Animator::Effect::Chase, 0xFF0000);  // Red chase
            break;
        case WS2812Animator::Effect::Blink:
            m_animator->setEffect(WS2812Animator::Effect::Blink, 0xFFFFFF);  // White blink
            break;
        case WS2812Animator::Effect::Larson:
            m_animator->setEffect(WS2812Animator::Effect::Larson, 0xFF0000);  // Red larson scanner
            break;
        default:
            m_animator->setEffect(WS2812Animator::Effect::SolidColor, 0x404040);  // Dim white fallback
            break;
    }
    
    // Update animation
    m_animator->tick();
}

bool WS2812TestThread::isTimeToChangeEffect()
{
    uint32_t currentTime = os_get_elapsed_time_msec();
    return (currentTime - m_effectStartTime) >= m_effectDuration;
}

const char* WS2812TestThread::getEffectName(WS2812Animator::Effect effect)
{
    switch (effect) {
        case WS2812Animator::Effect::SolidColor: return "Solid Color";
        case WS2812Animator::Effect::Rainbow: return "Rainbow";
        case WS2812Animator::Effect::Breath: return "Breathing";
        case WS2812Animator::Effect::Chase: return "Chase";
        case WS2812Animator::Effect::Blink: return "Blink";
        case WS2812Animator::Effect::Larson: return "Larson";
        default: return "Unknown";
    }
}
