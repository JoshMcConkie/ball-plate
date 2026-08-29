import json
import sys
from pathlib import Path

Import("env")

project_dir = Path(env.subst("$PROJECT_DIR"))
repo_root = project_dir.parent.parent
scripts_dir = project_dir / "scripts"
sys.path.insert(0, str(scripts_dir))

from config_codegen import ConfigError, render_header

config_path = repo_root / "data" / "system_config.json"
generated_dir = Path(env.subst("$BUILD_DIR")) / "generated"
header_path = generated_dir / "generated_system_config.hpp"

if not config_path.is_file():
    raise RuntimeError(f"Configuration file not found: {config_path}")

try:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    production = env.subst("$PIOENV") != "calibrate-servos"
    header = render_header(config, production=production)
except (json.JSONDecodeError, ConfigError) as exc:
    raise RuntimeError(f"Invalid firmware configuration: {exc}") from exc

generated_dir.mkdir(parents=True, exist_ok=True)

if not header_path.exists() or header_path.read_text() != header:
    header_path.write_text(header)

env.Append(CPPPATH=[str(generated_dir)])
env.Replace(MONITOR_SPEED=str(config["serial"]["baud_rate"]))
