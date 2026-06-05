#include "dev/coralnpu/uart_console.hh"

#include <cstdio>

#include "mem/packet.hh"
#include "mem/packet_access.hh"

namespace gem5 {

UartConsole::UartConsole(const Params &p)
    : BasicPioDevice(p, p.pio_size)
{}

Tick
UartConsole::read(PacketPtr pkt)
{
    // TX-ready / no RX FIFO — always return 0.
    pkt->setLE<uint64_t>(0);
    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
UartConsole::write(PacketPtr pkt)
{
    if (pkt->getSize() >= 1) {
        char c = static_cast<char>(pkt->getLE<uint8_t>());
        putchar(c);
        if (c == '\n')
            fflush(stdout);
    }
    pkt->makeAtomicResponse();
    return pioDelay;
}

} // namespace gem5
