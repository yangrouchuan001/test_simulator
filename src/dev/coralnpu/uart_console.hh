#pragma once

#include "dev/io_device.hh"
#include "params/UartConsole.hh"

namespace gem5 {

/**
 * UartConsole — minimal MMIO character-output device for CoralNPU simulation.
 *
 * Firmware writes a single byte to the mapped address (default 0xFFFFFFF8).
 * The device prints that byte to the host's stdout, emulating a UART TX
 * register.  Reads return 0 (TX-ready / no RX FIFO).
 */
class UartConsole : public BasicPioDevice
{
  public:
    PARAMS(UartConsole);
    UartConsole(const Params &p);

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
};

} // namespace gem5
