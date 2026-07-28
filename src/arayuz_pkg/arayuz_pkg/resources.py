from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parent


def resource_path(name):
    return PACKAGE_DIR / name
