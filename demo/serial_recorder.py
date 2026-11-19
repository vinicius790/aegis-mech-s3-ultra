# -*- coding: utf-8 -*-
"""Interactive Bench Session Recorder (Legacy CSV @palis.dev + Actuator Logs + Replay)."""

import csv
import threading
from pathlib import Path
from typing import List, Dict, Any

try:
    import serial  # type: ignore
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


class SerialRecorder:
    """Gravador thread-safe para sessoes de bancada (data/raw/ ou offline replay)."""

    def __init__(self, port: str = "COM3", baud: int = 921600):
        self.port = port
        self.baud = baud
        self.has_serial = HAS_SERIAL
        self._lock = threading.Lock()
        self._rows: List[Dict[str, Any]] = []

    def push_sample(self, sample: Dict[str, Any]):
        with self._lock:
            self._rows.append(dict(sample))

    def record_session(self, output_csv: str, rows: List[Dict[str, Any]] | None = None) -> int:
        out = Path(output_csv)
        out.parent.mkdir(parents=True, exist_ok=True)
        with self._lock:
            data = list(rows) if rows is not None else list(self._rows)
        if not data:
            return 0
        fields = list(data[0].keys())
        with open(out, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(data)
        return len(data)
