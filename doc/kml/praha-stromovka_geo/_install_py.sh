#!/bin/bash
# Linux counterpart of buchlovice-zpark_geo/_install_py.bat: venv in doc/kml/.venv with png+pyqrcode
cd "$(dirname "$0")/.."
python3 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install pypng pyqrcode
