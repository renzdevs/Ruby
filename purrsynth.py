"""
audio.purrsynth
===============
PurrSynth v2.1 interface.
Communicates over UART. Supports named clips + real-time parameter control.

Clip library (built-in to PurrSynth firmware):
    startup_chime       boot sequence chime
    shutdown_purr       soft farewell purr
    greeting_warm       chirpy meow for known faces
    greeting_cautious   soft trill for unknown faces
    purr                continuous purr (loops until stop())
    purr_loud           louder, more intense purr
    chirp               single bird-like chirp
    warning_hiss        defensive hiss
    playful_trill       rolling r trill during play

Protocol: UART text commands terminated with CRLF
    PLAY <clip_name> [LOOP]   -- play a clip
    STOP                       -- stop current playback
    VOL <0-100>                -- set volume
    PARAM <key> <value>        -- set synthesis parameter
    STATUS                     -- returns: PLAYING <clip> | IDLE
"""

import threading
import time
from typing import Optional

import serial


class PurrSynth:

    VALID_CLIPS = {
        "startup_chime", "shutdown_purr", "greeting_warm",
        "greeting_cautious", "purr", "purr_loud", "chirp",
        "warning_hiss", "playful_trill",
    }

    def __init__(self, port: str = "/dev/ttyUSB1", baud: int = 115200,
                 voice_profile: str = "ruby_v2", volume: float = 0.7,
                 sample_rate_hz: int = 22050, **_):
        self._port    = port
        self._baud    = baud
        self._profile = voice_profile
        self._volume  = int(volume * 100)
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._current_clip: Optional[str] = None

    def connect(self):
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=1.0)
            time.sleep(0.1)  # let PurrSynth boot
            self._send(f"PROFILE {self._profile}")
            self._send(f"VOL {self._volume}")
        except serial.SerialException:
            self._ser = None  # simulation mode

    def disconnect(self):
        if self._ser:
            self.stop()
            self._ser.close()

    def play(self, clip: str, loop: bool = False):
        if clip not in self.VALID_CLIPS:
            return
        self._current_clip = clip
        cmd = f"PLAY {clip}" + (" LOOP" if loop else "")
        self._send(cmd)

    def stop(self):
        self._current_clip = None
        self._send("STOP")

    def set_volume(self, volume: float):
        self._volume = int(max(0.0, min(1.0, volume)) * 100)
        self._send(f"VOL {self._volume}")

    def is_playing(self) -> bool:
        resp = self._query("STATUS")
        return resp.startswith("PLAYING")

    def _send(self, cmd: str):
        if self._ser:
            with self._lock:
                self._ser.write((cmd + "\r\n").encode())

    def _query(self, cmd: str) -> str:
        if not self._ser:
            return "IDLE"
        with self._lock:
            self._ser.write((cmd + "\r\n").encode())
            resp = self._ser.readline().decode().strip()
        return resp
