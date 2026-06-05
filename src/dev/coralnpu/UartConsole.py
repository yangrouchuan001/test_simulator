from m5.objects.Device import BasicPioDevice
from m5.params import Param


class UartConsole(BasicPioDevice):
    """
    Minimal MMIO UART-TX console for CoralNPU gem5 simulation.

    Maps a small address window (default: 0xFFFFFFF8, size 8 B).
    A byte write to any address in the window is printed to host stdout,
    emulating the firmware's UART character-output idiom:
        sb  a0, 0xFFFFFFF8
    """

    type = "UartConsole"
    cxx_header = "dev/coralnpu/uart_console.hh"
    cxx_class = "gem5::UartConsole"

    pio_size = Param.Addr(8, "Size of the MMIO window in bytes")
