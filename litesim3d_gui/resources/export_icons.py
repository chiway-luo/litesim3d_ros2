#!/usr/bin/env python3
"""Export the SVG master at native icon sizes; requires PyQt5 with QtSvg."""
import argparse
import os
from pathlib import Path
import struct

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QGuiApplication, QImage, QPainter
from PyQt5.QtSvg import QSvgRenderer


SIZES = (16, 24, 32, 48, 64, 128, 256, 512, 1024)
ICNS_TYPES = ((b"icp4", 16), (b"icp5", 32), (b"icp6", 64),
              (b"ic07", 128), (b"ic08", 256), (b"ic09", 512),
              (b"ic10", 1024), (b"ic11", 32), (b"ic12", 64),
              (b"ic13", 256), (b"ic14", 512))


def main():
    icon_dir = Path(__file__).resolve().parent / "icons"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=icon_dir / "litesim3d.svg")
    parser.add_argument("--output-dir", type=Path, default=icon_dir)
    args = parser.parse_args()
    app = QGuiApplication([])
    renderer = QSvgRenderer(str(args.source))
    if not renderer.isValid():
        parser.error(f"Invalid SVG: {args.source}")
    if renderer.viewBoxF().width() != renderer.viewBoxF().height():
        parser.error("The icon master must have a square viewBox")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for size in SIZES:
        image = QImage(size, size, QImage.Format_ARGB32_Premultiplied)
        image.fill(Qt.transparent)
        painter = QPainter(image)
        painter.setRenderHint(QPainter.Antialiasing)
        renderer.render(painter)
        painter.end()
        target = args.output_dir / f"litesim3d-{size}.png"
        if not image.save(str(target), "PNG"):
            raise RuntimeError(f"Cannot write {target}")

    # Package the unmodified PNG payloads into standard and Retina ICNS entries.
    entries = [(kind, (args.output_dir / f"litesim3d-{size}.png").read_bytes())
               for kind, size in ICNS_TYPES]
    toc = b"".join(kind + struct.pack(">I", len(data) + 8)
                   for kind, data in entries)
    body = b"TOC " + struct.pack(">I", len(toc) + 8) + toc
    body += b"".join(kind + struct.pack(">I", len(data) + 8) + data
                     for kind, data in entries)
    target = args.output_dir / "LiteSim3D.icns"
    target.write_bytes(b"icns" + struct.pack(">I", len(body) + 8) + body)
    print(f"Exported {len(SIZES)} PNGs and {target.name} from {args.source.name}")
    return app


if __name__ == "__main__":
    main()
