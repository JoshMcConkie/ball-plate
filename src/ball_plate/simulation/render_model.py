from pathlib import Path
import json

import mujoco
from jinja2 import Environment, FileSystemLoader, StrictUndefined

PROJECT_ROOT = Path(__file__).resolve().parents[3]

CONFIG_PATH = PROJECT_ROOT / "data" / "system_config.json"
TEMPLATE_DIRECTORY = PROJECT_ROOT / "simulation" / "models"
OUTPUT_PATH = PROJECT_ROOT / "simulation" / "generated" / "ball_plate.xml"

def main() -> None:
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))

    environment = Environment(
            loader=FileSystemLoader(TEMPLATE_DIRECTORY),
            undefined=StrictUndefined,
            autoescape=False,
            trim_blocks=True,
            lstrip_blocks=True,
            keep_trailing_newline=True,
    )

    template = environment.get_template("ball_plate.xml.j2")
    rendered_xml = template.render(config=config)

    # Compile before writing. This catches both invalid XML and invalid MJCF.
    mujoco.MjModel.from_xml_string(rendered_xml)

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(rendered_xml, encoding="utf-8")

    print(f"Generated and validated: {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
