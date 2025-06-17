#pragma once

#include "BaseThread.h"
#include "ws2812_cpp.hpp"
#include "ws2812_effects.hpp"
#include "hf_gpio_config.hpp"

/**
 * @brief Thread for testing WS2812 LED functionality
 * 
 * This thread runs various LED effects to test the WS2812 driver
 * and verify that the LED hardware is working correctly. It cycles
 * through different animations and colors using the RTOS BaseThread
 * infrastructure.
 */
class WS2812TestThread : public BaseThread {
private:
    WS2812Strip* strip;                    ///< WS2812 strip object
    WS2812Animator* animator;              ///< Animation controller
    uint8_t led_count;                     ///< Number of LEDs
    uint32_t update_interval_ms;           ///< Update interval in ms
    uint8_t brightness;                    ///< LED brightness (0-255)
    WS2812Animator::Effect current_effect; ///< Current animation effect
    uint32_t effect_start_time;            ///< When current effect started
    uint32_t effect_duration;              ///< How long to run each effect (ms)
    uint8_t effect_index;                  ///< Current effect index
    
    // List of effects to cycle through
    static const WS2812Animator::Effect effect_list[];
    static const size_t effect_count;

public:
    /**
     * @brief Constructor
     * @param ledCount Number of LEDs in the strip (default: 1)
     * @param updateIntervalMs Update interval in milliseconds (default: 20ms)
     */
    WS2812TestThread(uint8_t ledCount = 1, uint32_t updateIntervalMs = 20);

    /**
     * @brief Destructor
     */
    virtual ~WS2812TestThread();

    /**
     * @brief Set the LED brightness (0-255)
     * @param brightness Brightness level
     */
    void SetBrightness(uint8_t brightness);

    /**
     * @brief Set the update interval
     * @param intervalMs Update interval in milliseconds
     */
    void SetUpdateInterval(uint32_t intervalMs);

    /**
     * @brief Get current effect
     * @return Current animation effect
     */
    WS2812Animator::Effect GetCurrentEffect() const { return m_currentEffect; }

protected:
    /**
     * @brief Initialize the thread and WS2812 hardware
     * @return true if successful, false otherwise
     */
    virtual bool Initialize() noexcept override;

    /**
     * @brief Setup the WS2812 hardware before starting the main loop
     * @return true if successful, false otherwise
     */
    virtual bool Setup() noexcept override;

    /**
     * @brief Main thread step - runs the LED animation
     * @return delay in milliseconds before next step
     */
    virtual uint32_t Step() noexcept override;

    /**
     * @brief Cleanup when thread stops
     * @return true if cleanup successful
     */
    virtual bool Cleanup() noexcept override;

    /**
     * @brief Reset variables to initial state
     * @return true if reset successful
     */
    virtual bool ResetVariables() noexcept override;

private:
    /**
     * @brief Cycle to the next animation effect
     */
    void cycleToNextEffect();

    /**
     * @brief Run the current animation effect
     */
    void runCurrentEffect();

    /**
     * @brief Check if it's time to change effects
     * @return true if time to change effects
     */
    bool isTimeToChangeEffect();

    /**
     * @brief Get the name of an effect for logging
     * @param effect The effect to get the name for
     * @return Human-readable effect name
     */
    const char* getEffectName(WS2812Animator::Effect effect);

public:
    /**
     * @brief Set the duration for each effect
     * @param duration Duration in milliseconds
     */
    void SetEffectDuration(uint32_t duration);

    // Hardware objects
    WS2812Strip* m_strip;                    ///< WS2812 strip object
    WS2812Animator* m_animator;              ///< Animation controller
    
    // Configuration
    uint8_t m_ledCount;                      ///< Number of LEDs
    uint32_t m_updateIntervalMs;             ///< Update interval in ms
    uint8_t m_brightness;                    ///< LED brightness (0-255)
    
    // Animation state
    WS2812Animator::Effect m_currentEffect;  ///< Current animation effect
    uint64_t m_effectStartTime;              ///< When current effect started (microseconds)
    uint32_t m_effectDuration;               ///< How long to run each effect (ms)
    uint8_t m_effectIndex;                   ///< Current effect index
    uint32_t m_cycleCount;                   ///< Number of animation cycles completed
    
    // List of effects to cycle through
    static const WS2812Animator::Effect s_effectList[];
    static const size_t s_effectCount;
    
    // Thread stack
    static constexpr size_t STACK_SIZE = 4096;
    uint8_t m_threadStack[STACK_SIZE];
};
