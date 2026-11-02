"""vortex_canopen.frame_record: the Python twin of hf-core HfCanFrameRecord.h.

GOLDEN is byte for byte the kGolden datagram of hf-core
tests/host/test_frame_record.cpp, so both codecs agree on the wire format
(design §7.2). Needs no node process.
"""

from __future__ import annotations

import pytest

from conftest import REPO  # noqa: F401  (puts tools/vortex_canopen on sys.path)
from vortex_canopen import frame_record as fr

GOLDEN = bytes([
    0x01, 0x00, 0x02, 0x01, 0x0D, 0x0C, 0x0B, 0x0A,
    0x05, 0x06, 0x00, 0x00, 0x00, 0x04, 0x40, 0x00, 0x10, 0x00,
    0xEF, 0xCD, 0xAB, 0xC1, 0x00, 0x00,
    0x23, 0x01, 0x00, 0x00, 0x0C, 0x0C, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
])

FRAMES = [
    fr.Frame(0x605, bytes([0x40, 0x00, 0x10, 0x00])),
    fr.Frame(0x1ABCDEF, b"", fr.FLAG_EXT | fr.FLAG_RTR),
    fr.Frame(0x123, bytes(range(12)), fr.FLAG_FDF | fr.FLAG_BRS),
]


def test_golden_encode():
    assert fr.encode_datagram(FRAMES, seq=0x0102, epoch=0x0A0B0C0D) == GOLDEN


def test_golden_decode():
    header, frames = fr.decode_datagram(GOLDEN)
    assert (header.seq, header.epoch) == (0x0102, 0x0A0B0C0D)
    assert frames == FRAMES


@pytest.mark.parametrize("n", range(9))
def test_round_trip_classic_lengths(n):
    f = fr.Frame(0x7FF, bytes(range(0xA0, 0xA0 + n)))
    assert fr.decode_datagram(fr.encode_datagram([f]))[1] == [f]


@pytest.mark.parametrize(
    "mutate",
    [
        lambda b: b[:7],                      # truncated header
        lambda b: b"\x02" + b[1:],            # version
        lambda b: b[:-1],                     # truncated data
        lambda b: b[:13] + b"\x41" + b[14:],  # record length 65
        lambda b: b[:12] + b"\x01" + b[13:],  # EXT in record flags
    ],
    ids=["header", "version", "data", "length", "flags"],
)
def test_malformed_datagrams_are_rejected(mutate):
    with pytest.raises(fr.DecodeError):
        fr.decode_datagram(mutate(GOLDEN))


def test_encode_refuses_malformed_frames():
    for f in (fr.Frame(0x800), fr.Frame(0x100, b"", fr.FLAG_BRS), fr.Frame(0x100, bytes(9)),
              fr.Frame(0x100, bytes(9), fr.FLAG_FDF)):
        with pytest.raises(ValueError):
            fr.encode_record(f)


def test_stream_decoder_reassembles_split_datagrams():
    stream = fr.frame_stream(GOLDEN) + fr.frame_stream(fr.encode_datagram([FRAMES[0]], seq=3))
    dec = fr.StreamDecoder()
    out = []
    for i in range(0, len(stream), 5):
        out.extend(dec.feed(stream[i:i + 5]))
    assert out[0] == GOLDEN and len(out) == 2
