# greaseweazle/tools/diag/__init__.py
#
# Greaseweazle control script: Interactive live disk/drive diagnostic.
#
# Based on the work of Keir Fraser
#
# This is free and unencumbered software released into the public domain.
# See the file COPYING for more details, or visit <http://unlicense.org>.

description = "Interactive live disk/drive diagnostic."

import os, sys, time
from typing import List, NamedTuple, Optional, Tuple

from greaseweazle import error
from greaseweazle import usb as USB
# See decode.py for why codec.codec must be imported before ibm.ibm.
from greaseweazle.codec import codec  # noqa: F401
from greaseweazle.codec.ibm.ibm import Mode
from greaseweazle.tools import util
from greaseweazle.tools.delays import Delays
from greaseweazle.tools.diag import pinmap, decode, keyboard, batch

# ANSI color for the live sector count: bright green on a complete read,
# bright red otherwise. Windows 10+ consoles support these once virtual-
# terminal processing is enabled (see enable_vt_colors); POSIX terminals
# handle them natively.
_GREEN = '\x1b[92m'
_RED = '\x1b[91m'
_RESET = '\x1b[0m'


def enable_vt_colors() -> None:
    """Turn on ANSI escape handling in the Windows console (no-op if it
    isn't a real console, such as output redirected to a file)."""
    if os.name != 'nt':
        return
    try:
        import ctypes
        kernel32 = ctypes.windll.kernel32
        handle = kernel32.GetStdHandle(-11)  # STD_OUTPUT_HANDLE
        mode = ctypes.c_uint32()
        if not kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
            return
        # ENABLE_VIRTUAL_TERMINAL_PROCESSING = 0x0004
        kernel32.SetConsoleMode(handle, mode.value | 0x0004)
    except Exception:
        pass  # colors are cosmetic -- never fail the session over them

KEYLEGEND = """\
Keys: 0-9=goto track N0  +/-/<-/->=step 1  r=recalibrate
      h=head  m=motor  s=drive-select  d=density-select  q/Esc=quit"""

# (rate_kbps, nominal_rpm) -> (sectors/track, description). Sector count
# depends on rate *and* rpm together -- for example 500kbps is 15 sec/trk at
# 360rpm (1.2MB 5.25" HD) but 18 sec/trk at 300rpm (1.44MB 3.5" HD) -- so
# rate alone can't disambiguate it. Used both for the startup cheat sheet
# and to guess --secs when the user doesn't supply it.
STANDARD_FORMATS = {
    (250, 300): (9, '360KB 5.25" DD / 720KB 3.5" DD'),
    # 300kbps is the odd one: a 1.2MB 5.25" HD drive spins at 360rpm, so
    # reading a 300rpm-written DD disk in it scales the data rate up by
    # 360/300 = 1.2 (250 -> 300kbps) while the content stays 9 sec/trk.
    (300, 360): (9, 'DD disk in 1.2MB 5.25" HD drive @ 360rpm'),
    (500, 360): (15, '1.2MB 5.25" HD'),
    (500, 300): (18, '1.44MB 3.5" HD'),
    (1000, 300): (36, '2.88MB 3.5" ED'),
}


def cheatsheet() -> str:
    lines = ['Typical formats (rate @ rpm = sec/trk):']
    for (rate, rpm), (secs, desc) in sorted(STANDARD_FORMATS.items()):
        lines.append('  %5d kbps @ %3d rpm = %2d sec/trk  (%s)' %
                     (rate, rpm, secs, desc))
    return '\n'.join(lines)


def guess_secs(rate: int, rpm: Optional[float]) -> Optional[int]:
    if rpm is None:
        return None
    nominal_rpm = 360 if rpm >= 330 else 300
    entry = STANDARD_FORMATS.get((rate, nominal_rpm))
    return entry[0] if entry else None


class State:
    def __init__(self, args, delays: Delays) -> None:
        self.args = args
        self.delays = delays
        self.cyl = 0
        self.head = 0
        self.motor = True
        self.selected = True  # util.with_drive_selected() selects before run()
        self.density = False
        self.last_rpm: Optional[float] = None  # self-corrects the read window
        self.err_streak = 0  # consecutive ticks with no index (for recovery)
        # Set in --batch mode. While it is None, notices print as text.
        self.chan: Optional[batch.Channel] = None


def notice(st: State, msg: str, level: str = 'info') -> None:
    """Tell the user something out-of-band: a failed seek, a recalibration.

    Interactively that is a printed line in among the status lines. Under
    --batch it becomes an event record, so a front-end can surface it in its
    own UI instead of the caller having to scrape stderr for it.
    """
    if st.chan is not None:
        st.chan.send(batch.event_record(msg, level))
    else:
        print(msg)


def sync_density_pin(usb: USB.Unit, st: State) -> None:
    # Pin 2 is an output we drive ourselves -- there's no hardware
    # read-back for it (see pinmap.py), so we can't just trust that
    # whatever level the firmware defaults to after a reset matches
    # st.density. Force a real transition (away, then back) so the GW is
    # actually driving the level the status line claims, rather than a
    # same-value set_pin() silently being a no-op against a stale register.
    usb.set_pin(pinmap.DENSITY_SELECT_PIN, not st.density)
    usb.set_pin(pinmap.DENSITY_SELECT_PIN, st.density)


def try_seek(usb: USB.Unit, st: State, new_cyl: int) -> None:
    # No upper clamp -- the user can deliberately probe past the declared
    # --cyls (for example to find a drive's real mechanical limit). usb.seek()
    # itself rejects nonsense values, and a too-far seek on real hardware
    # surfaces as a CmdError/Fatal we catch below rather than a crash.
    new_cyl = max(0, new_cyl)
    # Double-step: an 80-track drive reading a 40-track disk moves two
    # physical cylinders per logical track. st.cyl stays *logical* (that's
    # what the sector IDAMs and the decoder compare against). Only the
    # physical seek target is doubled.
    phys_cyl = new_cyl * 2 if st.args.double_step else new_cyl
    try:
        usb.seek(phys_cyl, st.head)
        st.cyl = new_cyl
    except (error.Fatal, USB.CmdError) as e:
        notice(st, str(e), 'error')
        return
    if st.args.gen_tg43:
        st.density = st.cyl < pinmap.TG43_TRACK_THRESHOLD
        usb.set_pin(pinmap.DENSITY_SELECT_PIN, st.density)


# Cap on recalibration steps -- more cylinders than any real drive has, so
# a genuinely stuck head or dead TK0 sensor is reported instead of looping
# forever.
MAX_RECAL_STEPS = 80


def recalibrate(usb: USB.Unit, st: State) -> None:
    notice(st, 'Recalibrating to track 0')
    prior = st.cyl

    # A previous failed/aborted seek can leave the firmware's internal
    # cylinder counter out of sync with the real head position, so a plain
    # seek(0) computes a zero (or wrong) step delta and never physically
    # moves the head -- this is exactly the state "gw reset" has always
    # been observed to clear. power_on_reset() (Cmd.Reset) wipes that
    # internal state, so redo the per-session setup it also resets.
    try:
        usb.power_on_reset()
        st.delays.update()  # power_on_reset() wipes step/settle/etc back to
                            # firmware defaults -- restore the session's delays
        usb.set_bus_type(st.args.drive.bus.value)
        # Respect a manual 's' deselect -- don't silently re-select just
        # because the reset forces the bus type to be reasserted.
        if st.selected:
            usb.drive_select(st.args.drive.unit_id)
        else:
            usb.drive_deselect()
        usb.drive_motor(st.args.drive.unit_id, st.motor)
        sync_density_pin(usb, st)
    except USB.CmdError as e:
        notice(st, 'Recalibration reset failed: %s' % e, 'error')
        return

    for cyl in range(0, -MAX_RECAL_STEPS, -1):
        # Negative cylinders bypass usb.seek()'s own TRK0 check (it only
        # validates when target==0), so this always "succeeds" while still
        # physically stepping the head one track further each time -- a
        # fallback in case the reset alone doesn't fully re-home the head.
        try:
            usb.seek(cyl, st.head)
        except (error.Fatal, USB.CmdError):
            pass
        try:
            trk0 = not usb.get_pin(pinmap.TK0_PIN)
        except USB.CmdError:
            trk0 = False
        if trk0:
            st.cyl = 0
            try_seek(usb, st, prior)
            return
    notice(st, 'Track 0 signal never asserted after %d steps -- '
           'bad TK0 sensor or heads stuck?' % MAX_RECAL_STEPS, 'error')
    # prior is very likely 0 here (that's the common way this loop gets
    # triggered) -- don't re-run the same failing seek(0) and dump the raw
    # firmware error a second time right after our own diagnostic.
    if prior != 0:
        try_seek(usb, st, prior)


# The state changes behind the interactive keys, as explicit setters. The key
# handler drives them as toggles; --batch drives them to an absolute value, so
# a front-end that has its own buttons can set what it wants rather than
# toggling blind and hoping its idea of the current state is still right.

def set_head(usb: USB.Unit, st: State, head: int) -> None:
    if st.args.heads != 2 or head == st.head:
        return
    st.head = head
    # usb.seek() is what actually emits the head-select command. Just
    # flipping st.head leaves the device reading the *old* head until the
    # next physical step. Re-seek the current cylinder so the new head takes
    # effect immediately (no movement, same cyl).
    try_seek(usb, st, st.cyl)


def set_motor(usb: USB.Unit, st: State, on: bool) -> None:
    st.motor = on
    usb.drive_motor(st.args.drive.unit_id, on)


def set_select(usb: USB.Unit, st: State, on: bool) -> None:
    # Deliberately independent of the motor: some drives gate their head
    # load/unload solenoid off drive-select rather than motor-on, so this
    # lets that be tested on its own, with the motor left running (or not)
    # either way.
    st.selected = on
    if on:
        usb.drive_select(st.args.drive.unit_id)
    else:
        usb.drive_deselect()


def set_density(usb: USB.Unit, st: State, level: bool) -> None:
    if st.args.gen_tg43:  # pin 2 is auto-tracked -- leave it alone
        return
    st.density = level
    usb.set_pin(pinmap.DENSITY_SELECT_PIN, level)


def handle_key(usb: USB.Unit, st: State, key: Optional[str]) -> bool:
    """Returns False to request quit."""

    if key in ('q', 'esc'):
        return False
    elif key in ('left', ',', '-'):
        try_seek(usb, st, st.cyl - 1)
    elif key in ('right', '.', '+'):
        try_seek(usb, st, st.cyl + 1)
    elif key is not None and key.isdigit():
        try_seek(usb, st, int(key) * 10)
    elif key == 'h':
        set_head(usb, st, 1 - st.head)
    elif key == 'r':
        recalibrate(usb, st)
    elif key == 'm':
        set_motor(usb, st, not st.motor)
    elif key == 's':
        set_select(usb, st, not st.selected)
    elif key == 'd':
        set_density(usb, st, not st.density)
    return True


def drive_label(drive: util.Drive) -> str:
    if drive.bus == USB.BusType.IBMPC:
        return 'B' if drive.unit_id == 1 else 'A'
    return str(drive.unit_id)


class Reading(NamedTuple):
    """One tick's worth of measurements, before any formatting.

    Split out from the status line so the human-readable output and the
    --batch JSON record are two renderings of the same sampled data rather
    than two separate paths that can drift apart. Pin levels are the raw
    electrical levels (True = high); this interface is active-low, so the
    derived booleans alongside them carry the meaning.
    """
    drive: str
    cyl: int
    head: int
    rpm: Optional[float]        # None when there was no reading this tick
    motor: bool                 # False => rpm is 'off' rather than an error
    sect: int
    secs: Optional[int]         # expected count: given, guessed, or unknown
    off_track: List[Tuple[int, int]]
    selected: bool
    density: bool               # level we are driving on the density pin
    wp: Optional[bool]          # None when the pin could not be read back
    tk0: Optional[bool]
    dc: Optional[bool]


def sample(usb: USB.Unit, st: State) -> Reading:
    """Poll the pins and decode one flux capture from the current track."""

    args = st.args

    levels: dict = {}
    for label, pin, _ambiguous in pinmap.SIGNALS:
        try:
            levels[label] = usb.get_pin(pin)
        except USB.CmdError:
            levels[label] = None

    rpm_val, sect = None, 0
    off_track: List[Tuple[int, int]] = []
    if st.motor:
        # Bound the capture by *time*, not by index pulses: with no disk
        # inserted there is never an index pulse, so usb.read_track(revs=N)
        # would block waiting for one that will never come. Sizing the
        # window from the last known RPM (falling back to --rpm, then a
        # generic guess) keeps it self-correcting once a real disk is in.
        assumed_rpm = args.rpm or st.last_rpm or 250.0
        # Cap the assumed speed at 400rpm so the window is always at least
        # 2 revs of the slowest standard drive. A stale/high last_rpm must
        # never size the window below one real revolution, or the read can
        # never catch a second index pulse to recover from.
        assumed_rpm = min(assumed_rpm, 400.0)
        ticks = int(usb.sample_freq * (60 / assumed_rpm) * 2.0)
        try:
            flux = usb.read_track(revs=0, ticks=ticks)
            # Need two index pulses for a genuine index-to-index period.
            # index_list[-1] with only one index is the partial capture-
            # start-to-index time, which reads as a spuriously high RPM.
            # Storing that in last_rpm shrinks the next window into a
            # spiral the read never climbs back out of (this is what makes
            # RPM sometimes never come back after a disk is pulled and
            # reinserted). Fewer than two indexes means no reading.
            if len(flux.index_list) >= 2:
                tpr = flux.index_list[-1] / flux.sample_freq
                rpm_val = 60 / tpr
                st.last_rpm = rpm_val
                time_per_rev = (60 / args.rpm) if args.rpm else tpr
                mode = Mode.MFM if args.encoding == 'mfm' else Mode.FM
                sect, off_track = decode.decode_tick(
                    flux, st.cyl, st.head, mode, args.rate, time_per_rev)
        except USB.CmdError:
            pass
        except Exception:
            # No disk / no index found, or garbage flux -- never let this
            # kill the session, just report nothing decoded this tick.
            sect, off_track = 0, []

        if rpm_val is not None:
            st.err_streak = 0
        elif st.selected:
            # No index this tick while we are selected and meant to be
            # spinning. A brief run of these is just an empty or spinning-up
            # drive, but a sustained run can also mean the device stopped
            # reporting the index until re-poked (not simply an absent disk),
            # which is why RPM sometimes stays ERR after a disk is pulled and
            # reinserted. Every few ticks, re-assert select/motor and re-seek
            # the current cylinder to kick it, without spamming a command
            # every tick.
            st.err_streak += 1
            if st.err_streak % 4 == 0:
                try:
                    usb.drive_select(args.drive.unit_id)
                    usb.drive_motor(args.drive.unit_id, True)
                    try_seek(usb, st, st.cyl)
                except Exception:
                    pass

    return Reading(
        drive=drive_label(args.drive), cyl=st.cyl, head=st.head,
        rpm=rpm_val, motor=st.motor, sect=sect,
        secs=(args.secs if args.secs is not None
              else guess_secs(args.rate, rpm_val)),
        off_track=off_track, selected=st.selected, density=st.density,
        wp=levels['WP'], tk0=levels['TK0'], dc=levels['DC'])


def format_status(r: Reading) -> str:
    """The human-readable live status line."""

    ambiguous = {label: amb for label, _, amb in pinmap.SIGNALS}
    pin_of = {label: pin for label, pin, _ in pinmap.SIGNALS}

    def level_str(label: str, level: Optional[bool]) -> str:
        if level is None:
            return '?'
        return ('H' if level else 'L') + ('?' if ambiguous[label] else '')

    wp_str = level_str('WP', r.wp)
    if r.wp is not None:
        # This interface is active-low: WP asserted (L) == write-protected.
        wp_str += ' Unprot' if r.wp else ' Prot'

    tk0_str = level_str('TK0', r.tk0)
    if r.tk0 is not None:
        # Active-low: TK0 asserted (L) == head is at track 0.
        tk0_str += ' OFF' if r.tk0 else ' ON'

    if not r.motor:
        rpm_str = 'off'
    elif r.rpm is None:
        rpm_str = 'ERR'
    else:
        rpm_str = '%.2f' % r.rpm

    ot_str = ('NO' if not r.off_track else
             ','.join('T%d/S%d' % (c, n) for c, n in r.off_track))

    secs_str = str(r.secs) if r.secs is not None else '?'

    # Color the sector field: bright green on a complete read (every
    # expected sector decoded), bright red on anything short of that. Only
    # when the motor is spinning and we actually know the expected count --
    # a guessed/unknown '?' or a stopped motor leaves it uncolored.
    sect_field = 'S%d/%s' % (r.sect, secs_str)
    if r.motor and r.secs is not None:
        color = _GREEN if r.sect == r.secs else _RED
        sect_field = '%s%s%s' % (color, sect_field, _RESET)

    # Color RPM green within +-5 of either standard spindle speed (300rpm
    # for 5.25"/8", 360rpm for 1.2MB HD), red otherwise. Left uncolored
    # when there's no reading at all ('off'/'ERR').
    rpm_field = rpm_str
    if r.rpm is not None:
        in_range = 295 <= r.rpm <= 305 or 355 <= r.rpm <= 365
        rpm_field = '%s%s%s' % (_GREEN if in_range else _RED, rpm_str, _RESET)

    return ('Drive %s: T%d, H%d, RPM %s, %s, OT %s, SEL:%s, MOT:%s, '
            'WP:%s, TK0:%s, DEN %d:%s, DC%d:%s' %
            (r.drive, r.cyl, r.head, rpm_field, sect_field,
             ot_str, 'ON' if r.selected else 'OFF',
             'ON' if r.motor else 'OFF', wp_str, tk0_str,
             pinmap.DENSITY_SELECT_PIN, 'H' if r.density else 'L',
             pin_of['DC'], level_str('DC', r.dc)))


TICK = 0.5  # seconds between status updates


def run_batch(usb: USB.Unit, st: State) -> None:
    """The --batch loop: JSON records out, text commands in.

    Same measurements and same drive handling as the interactive loop -- only
    the input and output ends differ. See batch.py for the protocol.
    """

    chan = batch.Channel()
    st.chan = chan  # from here on, notice() emits events rather than printing
    chan.start()
    chan.send(batch.hello_record(
        st.args, drive_label(st.args.drive),
        {label: pin for label, pin, _ in pinmap.SIGNALS}))

    sync_density_pin(usb, st)
    recalibrate(usb, st)

    next_tick = time.monotonic()
    while True:
        line = chan.poll()
        if line == '':  # EOF: the front-end closed our stdin, so it is gone
            break
        if line is not None:
            if not do_command(usb, st, batch.parse(line)):
                break
            continue  # drain the whole queue before spending time on a read

        now = time.monotonic()
        if now >= next_tick:
            chan.send(batch.status_record(sample(usb, st)))
            next_tick = now + TICK
        else:
            time.sleep(0.02)

    chan.send({'t': 'bye'})


def do_command(usb: USB.Unit, st: State, cmd) -> bool:
    """Apply one parsed batch command. Returns False to request quit."""

    op = cmd['op']
    if op == 'quit':
        return False
    elif op == 'error':
        notice(st, cmd['msg'], 'error')
    elif op == 'goto':
        try_seek(usb, st, cmd['n'])
    elif op == 'step':
        try_seek(usb, st, st.cyl + cmd['n'])
    elif op == 'head':
        set_head(usb, st, cmd['n'])
    elif op == 'motor':
        set_motor(usb, st, cmd['v'])
    elif op == 'select':
        set_select(usb, st, cmd['v'])
    elif op == 'density':
        set_density(usb, st, cmd['v'])
    elif op == 'recal':
        recalibrate(usb, st)
    return True


def run(usb: USB.Unit, args, delays: Delays) -> None:

    st = State(args, delays)
    if args.batch:
        run_batch(usb, st)
        return

    enable_vt_colors()
    print(cheatsheet())
    print(KEYLEGEND)
    if args.gen_tg43:
        print('TG43 auto-tracking enabled on pin 2 (threshold T%d). '
              'The d key is disabled' % pinmap.TG43_TRACK_THRESHOLD)

    # Take the keyboard before touching the drive, so a terminal that can't
    # provide key input fails immediately rather than after a recalibrate.
    # The context manager restores the terminal on every exit path, including
    # Ctrl-C and an unexpected error.
    with keyboard.Keyboard() as kb:
        sync_density_pin(usb, st)  # make sure GW really drives what we assume
        recalibrate(usb, st)  # known starting position for the session

        next_tick = time.monotonic()
        while True:
            if kb.kbhit():
                key = kb.read_key()
                if key is not None and not handle_key(usb, st, key):
                    break

            now = time.monotonic()
            if now >= next_tick:
                print(format_status(sample(usb, st)))
                next_tick = now + TICK
            else:
                time.sleep(0.02)


def main(argv) -> None:

    epilog = (util.drive_desc)
    parser = util.ArgumentParser(usage='%(prog)s [options]', epilog=epilog)
    parser.add_argument("--device", help="greaseweazle device name")
    parser.add_argument("--drive", type=util.Drive(), default='A',
                        help="drive to diagnose")
    parser.add_argument("--cyls", type=util.min_int(1), default=84,
                        metavar="N", help="number of cylinders")
    parser.add_argument("--heads", type=int, choices=[1, 2], default=2,
                        help="number of heads")
    parser.add_argument("--double-step", action="store_true",
                        help="step two physical cylinders per track, for an "
                        "80-track drive reading a 40-track disk")
    parser.add_argument("--step-delay", type=util.uint, metavar="N",
                        help="Step Delay (usecs) for this session, as "
                        "'gw delays --step' (overrides the persisted "
                        "setting. Otherwise it is preserved across diag's "
                        "internal resets rather than reverting to the "
                        "firmware default)")
    parser.add_argument("--encoding", choices=['mfm', 'fm'], default='mfm',
                        help="track encoding")
    parser.add_argument("--rate", type=util.min_int(1), required=True,
                        metavar="KBPS", help="data rate, in kbps")
    parser.add_argument("--secs", type=util.min_int(1), default=None,
                        metavar="N", help="expected sectors per track "
                        "(omit to guess from rate/rpm for standard formats)")
    parser.add_argument("--rpm", type=float, default=None,
                        help="fixed spindle speed, in rpm "
                        "(omit to track the live measurement)")
    parser.add_argument("--gen-tg43", action="store_true",
                        help="auto-drive pin 2 as a TG43 signal for "
                        "8-inch drives (low from T%d up, high below), "
                        "matching --gen-tg43 in read/write/align. "
                        "Disables the d key, since pin 2 is then "
                        "under automatic control" % pinmap.TG43_TRACK_THRESHOLD)
    parser.add_argument("--batch", action="store_true",
                        help="machine-readable mode for a front-end hosting "
                        "the diagnostic in its own UI: one JSON record per "
                        "tick on stdout, plain-text commands on stdin, no "
                        "terminal needed")
    parser.description = description
    parser.prog += ' ' + argv[1]
    args = parser.parse_args(argv[2:])

    try:
        usb = util.usb_open(args.device)
        delays = Delays(usb)  # capture the persisted "gw delays" settings
        if args.step_delay is not None:
            delays.step = args.step_delay
        usb.power_on_reset()
        delays.update()  # power_on_reset() wiped them -- restore/apply now
        util.with_drive_selected(lambda: run(usb, args, delays), usb,
                                 args.drive)
    except USB.CmdError as err:
        print("Command Failed: %s" % err)


# Local variables:
# python-indent: 4
# End:
