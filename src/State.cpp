#include <State.h>

bool State::check(uint16_t flags)
{
    return (m_state & flags) == flags;
}

void State::set(uint16_t flags)
{
    m_state |= flags;
}

void State::clear(uint16_t flags)
{
    m_state &= ~flags;
}
