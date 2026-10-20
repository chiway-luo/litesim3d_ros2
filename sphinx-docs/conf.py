"""Read-only Sphinx configuration for generated copies of the repository docs."""
import importlib.util
import json
from pathlib import Path

info = json.loads((Path(__file__).parent / "build-info.json").read_text())
project = "LiteSim 3D"
author = "LiteSim 3D contributors"
version = info["version"]
release = info["label"]
language = "zh_CN"
extensions = ["myst_parser"]
source_suffix = {".md": "markdown"}
root_doc = "index"
exclude_patterns = ["_static", "_downloads"]
myst_enable_extensions = ["colon_fence"]
myst_heading_anchors = 6
html_theme = "sphinx_rtd_theme"
html_title = f"LiteSim 3D · {release}"
html_show_sourcelink = False
html_show_sphinx = False
html_show_copyright = False
html_theme_options = {
    "collapse_navigation": False,
    "navigation_depth": 3,
    "includehidden": True,
    "titles_only": True,
    "style_nav_header_background": "#244f69",
}
html_static_path = ["_static"]
html_css_files = ["docs.css"]
# No package installation is performed here. English/API-token search remains
# available without jieba; the build summary records that Chinese search is absent.
html_search_language = "zh" if importlib.util.find_spec("jieba") else "en"
