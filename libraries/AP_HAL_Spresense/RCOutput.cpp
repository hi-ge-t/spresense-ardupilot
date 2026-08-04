#include "RCOutput.h"

#if defined(__NuttX__)
#include <unistd.h>
#endif

void Spresense::RCOutput::init()
{
#if defined(__NuttX__)
    static constexpr char marker[] = "SPRESENSE_M1_OUTPUT=SHADOW_ONLY\n";
    (void)::write(STDOUT_FILENO, marker, sizeof(marker) - 1U);
#endif
}

void Spresense::RCOutput::set_freq(uint32_t channel_mask, uint16_t frequency_hz)
{
    (void)channel_mask;
    (void)frequency_hz;
}

uint16_t Spresense::RCOutput::get_freq(uint8_t channel)
{
    (void)channel;
    return 0U;
}

void Spresense::RCOutput::enable_ch(uint8_t channel)
{
    (void)channel;
}

void Spresense::RCOutput::disable_ch(uint8_t channel)
{
    (void)channel;
}

void Spresense::RCOutput::write(uint8_t channel, uint16_t period_us)
{
    (void)_guard.request_write(channel, period_us);
#if defined(__NuttX__)
    static bool rejection_reported;
    if (!rejection_reported) {
        rejection_reported = true;
        static constexpr char marker[] =
            "SPRESENSE_M1_OUTPUT=WRITE_REJECTED\n";
        (void)::write(STDOUT_FILENO, marker, sizeof(marker) - 1U);
    }
#endif
}

uint16_t Spresense::RCOutput::read(uint8_t channel)
{
    return _guard.shadow_period_us(channel);
}

void Spresense::RCOutput::read(uint16_t *period_us, uint8_t length)
{
    if (period_us != nullptr) {
        for (uint8_t channel = 0; channel < length; channel++) {
            period_us[channel] = read(channel);
        }
    }
}

void Spresense::RCOutput::cork()
{
    _guard.begin_shadow_frame();
}

void Spresense::RCOutput::push()
{
    _guard.commit_shadow_frame();
}

const Spresense::OutputGuard &Spresense::RCOutput::guard() const
{
    return _guard;
}
