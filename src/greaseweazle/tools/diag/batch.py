# greaseweazle/tools/diag/batch.py
#
# Machine-readable front end for the interactive diagnostic: newline-delimited
# JSON out, plain-text commands in, so another program can host the diagnostic
# in its own UI.
#
# Based on the work of Keir Fraser
#
# This is free and unencumbered software released into the public domain.
# See the file COPYING for more details, or visit <http://unlicense.org>.

"""The --batch protocol.

Records out (one compact JSON object per line, on real stdout):

  {"t":"hello", ...}    once at startup: the geometry and settings in force
  {"t":"status", ...}   one per tick, the same measurements the live status
                        line is rendered from
  {"t":"event", ...}    something worth telling the user about, such as a
                        failed seek or a recalibration
  {"t":"bye"}           last line before a clean exit

Commands in (one per line, on stdin). Unknown or malformed lines are
answered with an event rather than being ignored silently, so a front-end
with a typo finds out:

  goto N            seek to logical track N
  step N            step N tracks, negative to step outward
  head N            select head 0 or 1
  motor on|off      spindle motor
  select on|off     drive-select line, independent of the motor
  density on|off    level driven on the density-select pin
  recal             recalibrate to track 0, then return to the current track
  quit              shut down cleanly

The set-state commands take an absolute value rather than toggling, so a
front-end with its own on-screen controls can drive the drive to a known
state instead of toggling against a stale idea of the current one.

Commands are handled between ticks, so one that talks to the drive delays
the next status record until the drive answers. A recalibrate against a
drive whose TK0 never asserts is the slow case: it steps up to
MAX_RECAL_STEPS times before giving up, and a "quit" sent meanwhile is not
read until that finishes. A front-end that wants a hard deadline on exit
should close our stdin (which ends the session at the next poll) and fall
back to killing the process.

Why stdout and not stderr: cli.py points sys.stdout at sys.stderr so all
human logging shares one stream, leaving the real stdout clean. This uses
that clean stream, so a caller can read structured records off stdout while
warnings and tracebacks still go to stderr where a human expects them.
"""

import json, queue, sys, threading
from typing import Any, Dict, Optional


class Channel:
    """The JSON-out/commands-in pipe pair, plus the stdin reader thread."""

    def __init__(self) -> None:
        # sys.stdout has been pointed at stderr by cli.py; sys.__stdout__ is
        # still the real one. It can be None under a GUI host with no console,
        # in which case there is nothing to talk to.
        self.out = sys.__stdout__
        self.q: 'queue.Queue[Optional[str]]' = queue.Queue()
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        # A blocking readline on a daemon thread rather than select(): stdin
        # here is a pipe, and select() does not work on pipes on Windows.
        # Daemon so a front-end that stops sending commands without saying
        # "quit" cannot wedge our exit.
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self) -> None:
        try:
            for line in sys.stdin:
                self.q.put(line)
        except Exception:
            pass  # stdin closed or went away -- treated as EOF below
        self.q.put(None)  # EOF sentinel: the front-end has gone

    def send(self, record: Dict[str, Any]) -> None:
        if self.out is None:
            return
        try:
            self.out.write(json.dumps(record, separators=(',', ':')) + '\n')
            self.out.flush()  # a front-end is reading this line by line
        except (OSError, ValueError):
            pass  # the far end closed the pipe; the tick loop notices via EOF

    def poll(self) -> Optional[str]:
        """The next pending command line, '' on EOF, or None if none waiting."""
        try:
            line = self.q.get_nowait()
        except queue.Empty:
            return None
        return '' if line is None else line.strip()


def status_record(r) -> Dict[str, Any]:
    """A Reading as a flat JSON-friendly dict.

    Pin levels are the raw electrical level (true = high). Because the
    interface is active-low, and getting that backwards is the easy mistake
    to make in a front-end, the derived meaning is sent alongside rather than
    left for the caller to work out. Disk-change is raw-only on purpose: pin
    34's meaning genuinely varies by drive family (see pinmap.py).
    """
    return {
        't': 'status',
        'drive': r.drive,
        'cyl': r.cyl,
        'head': r.head,
        # null when no reading: motor false says which of the two it is.
        'rpm': None if r.rpm is None else round(r.rpm, 2),
        'motor': r.motor,
        'sect': r.sect,
        'secs': r.secs,  # null when unknown and not guessable
        'off_track': [[c, n] for c, n in r.off_track],
        'sel': r.selected,
        'density': r.density,
        'wp': r.wp,
        'write_protected': None if r.wp is None else not r.wp,
        'tk0': r.tk0,
        'at_track0': None if r.tk0 is None else not r.tk0,
        'dc': r.dc,
    }


def hello_record(args, drive: str, pins: Dict[str, int]) -> Dict[str, Any]:
    from greaseweazle import __version__
    return {
        't': 'hello',
        'protocol': 1,
        'version': __version__,
        # The display label ('A'/'B' on an IBM/PC bus, the unit number on a
        # Shugart one), matching the 'drive' field of every status record.
        'drive': drive,
        'cyls': args.cyls,
        'heads': args.heads,
        'rate': args.rate,
        'encoding': args.encoding,
        'secs': args.secs,
        'rpm': args.rpm,
        'double_step': args.double_step,
        'gen_tg43': args.gen_tg43,
        'pins': pins,
    }


def event_record(msg: str, level: str = 'info') -> Dict[str, Any]:
    return {'t': 'event', 'level': level, 'msg': msg}


def parse(line: str) -> Dict[str, Any]:
    """One command line into a dict, or {'op': 'error', 'msg': ...}."""

    parts = line.split()
    if not parts:
        return {'op': 'nop'}
    op, rest = parts[0].lower(), parts[1:]

    def bad(why: str) -> Dict[str, Any]:
        return {'op': 'error', 'msg': '%s: %s' % (line, why)}

    def as_int() -> Optional[int]:
        try:
            return int(rest[0])
        except (IndexError, ValueError):
            return None

    def as_bool() -> Optional[bool]:
        if not rest:
            return None
        word = rest[0].lower()
        if word in ('on', 'true', '1', 'high', 'h'):
            return True
        if word in ('off', 'false', '0', 'low', 'l'):
            return False
        return None

    if op in ('quit', 'exit'):
        return {'op': 'quit'}
    if op == 'recal':
        return {'op': 'recal'}
    if op in ('goto', 'step'):
        n = as_int()
        if n is None:
            return bad('expected a track number')
        return {'op': op, 'n': n}
    if op == 'head':
        n = as_int()
        if n not in (0, 1):
            return bad('expected head 0 or 1')
        return {'op': 'head', 'n': n}
    if op in ('motor', 'select', 'density'):
        v = as_bool()
        if v is None:
            return bad('expected on or off')
        return {'op': op, 'v': v}
    return bad('unknown command')


# Local variables:
# python-indent: 4
# End:
