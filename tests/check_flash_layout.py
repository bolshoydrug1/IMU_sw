"""Check the actual ELF load image, vector table and calibration reservation.

Usage: python tests/check_flash_layout.py build/Debug/IMU_sw_1.elf
Uses only Python's standard library. Does not connect to or program a device.
"""
from pathlib import Path
import struct
import sys


def check(path):
    data = Path(path).read_bytes()
    header = struct.unpack_from('<16sHHIIIIIHHHHHH', data)
    if header[0][:7] != b'\x7fELF\x01\x01\x01' or header[2] != 40:
        raise ValueError('Expected little-endian ELF32 for ARM')
    entry, phoff, shoff = header[4:7]
    phsize, phcount, shsize, shcount, shstrindex = header[9:14]
    reserved = [(0x08008000, 0x0800C000)]
    segments = []
    for i in range(phcount):
        kind, offset, vaddr, paddr, filesz, memsz, flags, align = struct.unpack_from(
            '<IIIIIIII', data, phoff + i * phsize)
        if kind == 1 and filesz:  # PT_LOAD: physical address is the programming address.
            for start, end in reserved:
                if paddr < end and paddr + filesz > start:
                    raise ValueError(f'Load segment overlaps calibration: {paddr:#010x}+{filesz:#x}')
            segments.append((paddr, paddr + filesz))

    sections = [struct.unpack_from('<IIIIIIIIII', data, shoff + i * shsize)
                for i in range(shcount)]
    strings = sections[shstrindex]
    names = data[strings[4]:strings[4] + strings[5]]

    def string_at(blob, offset):
        return blob[offset:blob.index(b'\0', offset)].decode('ascii')

    by_name = {string_at(names, s[0]): s for s in sections}
    vectors = by_name['.isr_vector']
    if vectors[3] != 0x08000000 or vectors[5] < 8:
        raise ValueError('Invalid boot vector table address/size')
    stack, reset = struct.unpack_from('<II', data, vectors[4])
    if stack != 0x20010000 or reset != entry or not reset & 1:
        raise ValueError(f'Invalid stack/reset vector: {stack:#x}, {reset:#x}, entry={entry:#x}')
    if not 0x0800C000 <= (entry & ~1) < 0x08040000:
        raise ValueError('Reset_Handler is outside application Flash')
    if by_name['.text'][3] != 0x0800C000:
        raise ValueError('Application text must start at 0x0800C000')
    for name, section in by_name.items():
        if section[2] & 2:  # SHF_ALLOC, including sections without file contents.
            for start, end in reserved:
                if section[5] and section[3] < end and section[3] + section[5] > start:
                    raise ValueError(f'Allocated section {name} overlaps calibration')
    for paddr, end in segments:
        if 0x08000000 <= paddr < 0x08040000 and end > 0x08040000:
            raise ValueError('Load image exceeds Flash capacity')

    symbols = {}
    for section in sections:
        if section[1] != 2:  # SHT_SYMTAB
            continue
        table = sections[section[6]]
        strings = data[table[4]:table[4] + table[5]]
        for offset in range(section[4], section[4] + section[5], section[9]):
            name, value, size, info, other, index = struct.unpack_from('<IIIBBH', data, offset)
            symbols[string_at(strings, name)] = value
    expected = {
        '__calib_flash_start__': 0x08008000,
        '__calib_flash_end__': 0x0800C000,
        '__calib_flash_size__': 0x4000,
    }
    for name, value in expected.items():
        if symbols.get(name) != value:
            raise ValueError(f'Invalid linker symbol {name}')
    print(f'{path}: OK; vectors=0x08000000, text=0x0800C000, reset={entry:#010x}')
    print('Calibration sector 2: no allocated sections or load-image bytes')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    for path in sys.argv[1:]:
        check(path)
