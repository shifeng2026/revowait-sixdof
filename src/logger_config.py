from __future__ import annotations

import logging
import sys
from datetime import datetime
from logging.handlers import TimedRotatingFileHandler
from pathlib import Path

_CONFIGURED = False


def configure_logging(
    level: int = logging.INFO,
    console_level: int = logging.WARNING,
) -> Path:
    """Configure shared console/file logging and return the active log path."""
    global _CONFIGURED

    log_dir = Path(__file__).resolve().parent / "logs"
    log_dir.mkdir(exist_ok=True)
    log_path = log_dir / f"sixaxis_{datetime.now():%Y%m%d}.log"

    root = logging.getLogger()
    root.setLevel(level)

    if not _CONFIGURED:
        fmt = logging.Formatter(
            "%(asctime)s %(levelname)s [%(name)s] [%(threadName)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )

        file_handler = TimedRotatingFileHandler(
            log_path,
            when="midnight",
            backupCount=14,
            encoding="utf-8",
        )
        file_handler.setFormatter(fmt)
        file_handler.setLevel(level)

        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setFormatter(fmt)
        console_handler.setLevel(console_level)

        root.addHandler(file_handler)
        root.addHandler(console_handler)
        _CONFIGURED = True

    return log_path


def get_logger(name: str) -> logging.Logger:
    configure_logging()
    return logging.getLogger(name)
