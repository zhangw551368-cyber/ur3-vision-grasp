from pathlib import Path

import yaml


def load_config(path):
    path = Path(path).expanduser().resolve()
    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    config["_config_path"] = str(path)
    config["_project_root"] = str(path.parent.parent)
    return config


def resolve_project_path(config, value):
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = Path(config["_project_root"]) / path
    return path.resolve()


def require_keys(mapping, keys, section):
    missing = [key for key in keys if key not in mapping]
    if missing:
        raise ValueError("{} missing required keys: {}".format(section, ", ".join(missing)))

