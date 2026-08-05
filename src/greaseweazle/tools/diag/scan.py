# greaseweazle/tools/diag/scan.py
#
# Surface scan: where every sector physically sits on every track.
#
# Based on the work of Keir Fraser
#
# This is free and unencumbered software released into the public domain.
# See the file COPYING for more details, or visit <http://unlicense.org>.

"""Measure each sector's angular position relative to the index pulse.

A normal read reports *which* sectors came back. This reports *where they
are*: the time from the index pulse at which each sector's address mark
passes the head, as a fraction of that revolution. That is the one thing a
sector-recovery report cannot tell you, and it is what makes rotational
problems visible — a drive whose speed wanders writes (or reads) sectors at
uneven angular spacing, and the unevenness shows up here as sectors drifting
away from their nominal positions.

The measurement comes from the PLL's own bitcell timings, so it reflects real
elapsed time rather than position in the bitstream: if the disk slows for part
of a revolution the bitcells stretch, the bit *count* stays the same, and only
the timing reveals it.

Decoding is done one revolution at a time. `IBMTrack.decode_flux` merges every
revolution into one deduplicated set of sectors, which is what you want for
recovering data and exactly what you don't want here, since it discards which
revolution a sector was seen in and therefore its timing.
"""

import struct
from typing import Dict, List, Optional, Tuple

from greaseweazle import usb as USB
from greaseweazle.codec import codec  # noqa: F401  (see decode.py on import order)
from greaseweazle.codec.ibm.ibm import (
    Mark, crc16, decode, fm_sync_prefix, mfm_sync, IBMTrack, Mode
)
from greaseweazle.track import PLLTrack


class SectorPosition:
    """One sector as it was found on one revolution."""

    def __init__(self, ident: int, cyl: int, head: int, size: int,
                 ok: bool, angle: float, seconds: float) -> None:
        self.ident = ident      # the 'R' of the IDAM: the sector's own number
        self.cyl = cyl          # 'C' from the IDAM, which may not match reality
        self.head = head        # 'H' from the IDAM
        self.size = size        # 'N': 128 << size bytes
        self.ok = ok            # data CRC checked out
        self.angle = angle      # position in the revolution, 0.0-1.0 from index
        self.seconds = seconds  # time from the index pulse

    def record(self) -> dict:
        return {
            'id': self.ident,
            'c': self.cyl,
            'h': self.head,
            'n': self.size,
            'ok': self.ok,
            'angle': round(self.angle, 5),
            'ms': round(self.seconds * 1e3, 4),
        }


def _cumulative(times: List[float]) -> List[float]:
    """Running total of bitcell durations, so a bit offset becomes a time."""
    out = [0.0]
    total = 0.0
    for t in times:
        total += t
        out.append(total)
    return out


def scan_revolution(bits, times, mode: Mode) -> List[SectorPosition]:
    """Every sector in one revolution's bitstream, with its timing.

    Mirrors `IBMTrack.mfm_decode_raw` / `fm_decode_raw` -- same sync search,
    same header layouts, same CRC checks -- but keeps each address mark's bit
    offset so its timing survives, and never merges revolutions. The two
    encodings are kept apart rather than parameterised: FM shifts past the
    sync prefix and validates a clock byte, and pretending the difference is
    just a couple of lengths is how you end up with plausible wrong answers.
    """
    period = sum(times)
    if period <= 0:
        return []
    cum = _cumulative(times)
    finder = _mfm_marks if mode is Mode.MFM else _fm_marks

    out: List[SectorPosition] = []
    for (s_idam, c, h, r, n, ok) in finder(bits):
        # Time the sector from its *address mark*: where the sector begins as
        # far as the disk is concerned.
        t = cum[min(s_idam, len(cum) - 1)]
        out.append(SectorPosition(r, c, h, n, ok, t / period, t))
    return out


def _mfm_marks(bits):
    """(idam_offset, c, h, r, n, data_ok) for each MFM sector found."""
    idam: Optional[Tuple[int, int, int, int, int]] = None
    for offs in bits.search(mfm_sync):
        if len(bits) < offs + 4 * 16:
            continue
        mark = decode(bits[offs + 3 * 16:offs + 4 * 16].tobytes())[0]

        if mark == Mark.IDAM:
            s, e = offs, offs + 10 * 16
            if len(bits) < e:
                continue
            b = decode(bits[s:e].tobytes())
            if crc16.new(b).crcValue != 0:
                continue  # a header we can't trust says nothing about where
            c, h, r, n = struct.unpack('>4x4B2x', b)
            idam = (s, c, h, r, n)

        elif mark in (Mark.DAM, Mark.DDAM):
            if idam is None or offs - (idam[0] + 10 * 16) > 1000:
                idam = None
                continue  # a data mark too far from its header to belong to it
            s_idam, c, h, r, n = idam
            idam = None
            e = offs + (4 + (128 << n) + 2) * 16
            ok = len(bits) >= e and \
                crc16.new(decode(bits[offs:e].tobytes())).crcValue == 0
            yield (s_idam, c, h, r, n, ok)


def _fm_marks(bits):
    """(idam_offset, c, h, r, n, data_ok) for each FM sector found."""
    idam: Optional[Tuple[int, int, int, int, int]] = None
    for offs in bits.search(fm_sync_prefix):
        offs += 16  # step over the sync prefix to the mark itself
        if len(bits) < offs + 1 * 16:
            continue
        mark = decode(bits[offs:offs + 1 * 16].tobytes())[0]
        # FM marks carry a distinctive clock pattern; without this check,
        # ordinary data can masquerade as an address mark.
        if decode(bits[offs - 1:offs + 1 * 16 - 1].tobytes())[0] != 0xc7:
            continue

        if mark == Mark.IDAM:
            s, e = offs, offs + 7 * 16
            if len(bits) < e:
                continue
            b = decode(bits[s:e].tobytes())
            if crc16.new(b).crcValue != 0:
                continue
            c, h, r, n = struct.unpack('>x4B2x', b)
            idam = (s, c, h, r, n)

        elif mark in (Mark.DAM, Mark.DDAM, Mark.DAM_TRS80_DIR):
            if idam is None or offs - (idam[0] + 7 * 16) > 1000:
                idam = None
                continue
            s_idam, c, h, r, n = idam
            idam = None
            e = offs + (1 + (128 << n) + 2) * 16
            ok = len(bits) >= e and \
                crc16.new(decode(bits[offs:e].tobytes())).crcValue == 0
            yield (s_idam, c, h, r, n, ok)


class TrackScan:
    """A whole track's worth of measurements, averaged over the revolutions."""

    def __init__(self, cyl: int, head: int) -> None:
        self.cyl = cyl
        self.head = head
        self.rpm: Optional[float] = None
        self.sectors: List[SectorPosition] = []
        self.revs = 0

    def record(self) -> dict:
        return {
            't': 'track',
            'cyl': self.cyl,
            'head': self.head,
            'rpm': None if self.rpm is None else round(self.rpm, 3),
            'revs': self.revs,
            'sectors': [s.record() for s in self.sectors],
        }


def scan_track(usb: USB.Unit, cyl: int, head: int, phys_cyl: int,
               rate_kbps: int, mode: Mode, revs: int) -> TrackScan:
    """Seek to a track and measure where its sectors are."""

    out = TrackScan(cyl, head)
    usb.seek(phys_cyl, head)
    flux = usb.read_track(revs=revs)
    flux.cue_at_index()
    raw = PLLTrack(clock=5e-4 / rate_kbps, data=flux, time_per_rev=None)

    periods: List[float] = []
    # Collect each sector's measurements across revolutions, keyed by its id.
    seen: Dict[int, List[SectorPosition]] = {}
    for rev in range(len(raw.revolutions)):
        bits, times = raw.get_revolution(rev)
        period = sum(times)
        if period <= 0:
            continue
        periods.append(period)
        out.revs += 1
        for s in scan_revolution(bits, times, mode):
            seen.setdefault(s.ident, []).append(s)

    if periods:
        out.rpm = 60 / (sum(periods) / len(periods))

    # Average each sector's angle over the revolutions it was seen in, which
    # takes most of the jitter out, and call a sector good if any revolution
    # read it cleanly -- the same "best of N" rule a retrying read uses.
    for ident in sorted(seen):
        group = seen[ident]
        angle = sum(s.angle for s in group) / len(group)
        seconds = sum(s.seconds for s in group) / len(group)
        first = group[0]
        out.sectors.append(SectorPosition(
            ident, first.cyl, first.head, first.size,
            any(s.ok for s in group), angle, seconds))
    out.sectors.sort(key=lambda s: s.angle)
    return out


# Local variables:
# python-indent: 4
# End:
