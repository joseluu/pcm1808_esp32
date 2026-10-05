# /// script
# dependencies = ["pyserial"]
# ///
"""Enregistre le flux audio 16 kHz de firmware_i2s_test dans un fichier WAV.

Usage : uv run tools/capture_wav.py COM15 10 capture.wav
(port, duree en secondes, fichier de sortie)
"""
import math
import struct
import sys
import time
import wave

import serial

MAGIC = bytes([0xA5, 0x5A, 0xC3, 0x3C])
HEADER_LEN = 8
RATE = 16000


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM15"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    out_path = sys.argv[3] if len(sys.argv) > 3 else "capture_16k.wav"

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.timeout = 0.5
    # DTR/RTS pilotent le reset sur l'USB-Serial/JTAG de l'ESP32-S3 : les
    # laisser inactifs a l'ouverture pour ne pas redemarrer la carte.
    ser.dtr = False
    ser.rts = False
    ser.open()

    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.write(b"S")
    print("Flux demande, enregistrement de %.1f s..." % duration)

    buf = bytearray()
    samples = []
    frames = 0
    lost = 0
    resyncs = 0
    last_seq = None
    end = time.time() + duration
    while time.time() < end:
        buf += ser.read(4096)
        while True:
            i = buf.find(MAGIC)
            if i < 0:
                del buf[:-3]
                break
            if i > 0:
                if frames > 0:
                    resyncs += 1
                del buf[:i]
            if len(buf) < HEADER_LEN:
                break
            seq, n = struct.unpack_from("<HH", buf, 4)
            if n > 1000:
                del buf[:1]
                resyncs += 1
                continue
            if len(buf) < HEADER_LEN + 2 * n:
                break
            samples.extend(struct.unpack_from("<%dh" % n, buf, HEADER_LEN))
            del buf[:HEADER_LEN + 2 * n]
            if last_seq is not None:
                lost += (seq - last_seq - 1) & 0xFFFF
            last_seq = seq
            frames += 1
    ser.close()

    if not samples:
        print("Aucune trame recue (firmware en attente de 'S' ? carte redemarree ?)")
        return 1

    with wave.open(out_path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(struct.pack("<%dh" % len(samples), *samples))

    rms = math.sqrt(sum(s * s for s in samples) / len(samples))
    peak = max(abs(s) for s in samples)
    print("Trames recues : %d, perdues : %d, resynchronisations : %d" % (frames, lost, resyncs))
    print("Echantillons : %d (%.2f s a 16 kHz)" % (len(samples), len(samples) / RATE))
    print("Niveau : rms %.1f dBFS, pic %.1f dBFS" % (
        20 * math.log10(max(rms, 1e-9) / 32768), 20 * math.log10(max(peak, 1) / 32768)))
    print("Fichier : %s" % out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
