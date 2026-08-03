#include "RCOutput.h"

#include <string.h>

void Spresense::RCOutput::init()
{
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
}

uint16_t Spresense::RCOutput::read(uint8_t channel)
{
    (void)channel;
    return 0U;
}

void Spresense::RCOutput::read(uint16_t *period_us, uint8_t length)
{
    if (period_us != nullptr) {
        memset(period_us, 0, static_cast<size_t>(length) * sizeof(period_us[0]));
    }
}

void Spresense::RCOutput::cork()
{
}

void Spresense::RCOutput::push()
{
}

const Spresense::OutputGuard &Spresense::RCOutput::guard() const
{
    return _guard;
}
