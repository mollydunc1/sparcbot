"""Minimal CSV writer behind WheelController.log().

You don't need to edit this file. Turn logging on by setting `log_file` in
sparcbot_bringup/config/sparcbot.yaml (wheel_controller), then call
self.log(...) inside WheelController.compute().
"""

import csv
from pathlib import Path


class DataLogger:

    def __init__(self, path: str):
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(self.path, 'w', newline='')
        self._writer = None
        self._rows = 0

    def write(self, values: dict) -> None:
        """Write one row. The first call's keys become the CSV header."""
        if self._writer is None:
            self._writer = csv.DictWriter(self._file, fieldnames=list(values))
            self._writer.writeheader()
        self._writer.writerow(values)
        self._rows += 1
        if self._rows % 100 == 0:
            self._file.flush()

    def close(self) -> None:
        if not self._file.closed:
            self._file.close()