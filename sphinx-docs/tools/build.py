#!/usr/bin/env python3
"""Generate a fresh Sphinx site without changing the repository Markdown."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
from urllib.parse import quote, unquote

ROOT = Path(__file__).resolve().parents[2]
SLUGS = {
    "docs/README.md": "index",
    "docs/版本与验证状态.md": "versions",
    "docs/tutorials/README.md": "tutorials/index",
    "docs/tutorials/双MID360从零搭建.md": "tutorials/dual-mid360-from-scratch",
    "docs/tutorials/阿克曼与全向轮ros2_control.md": "tutorials/ackermann-omni-control",
    "docs/tutorials/全部模型与第三方算法ros2_control.md": "tutorials/unified-control-plugins",
    "docs/tutorials/启动与控制.md": "tutorials/first-simulation",
    "docs/tutorials/添加IMU.md": "tutorials/add-imu",
    "docs/tutorials/ROS2接入.md": "tutorials/ros-client",
    "docs/usage/README.md": "manual/index",
    "docs/usage/安装与构建.md": "manual/installation",
    "docs/usage/接入与使用手册.md": "manual/operation",
    "docs/usage/模型接入.md": "manual/models",
    "docs/usage/模型组合与插件迁移.md": "manual/composition",
    "docs/usage/场景与地形.md": "manual/worlds",
    "docs/usage/传感器配置.md": "manual/sensors",
    "docs/usage/SDF与独立窗口.md": "manual/sdf-viewer",
    "docs/usage/多机器人.md": "manual/multiple-robots",
    "docs/usage/点云适配.md": "manual/cloud-adapter",
    "docs/usage/会话与重置.md": "manual/sessions",
    "docs/usage/记录与排障.md": "manual/troubleshooting",
    "docs/usage/目录与迁移.md": "manual/migration",
    "docs/usage/兼容配置.md": "manual/legacy",
    "docs/api/README.md": "reference/index",
    "docs/api/ROS2接口.md": "reference/ros2",
    "docs/api/消息与服务.md": "reference/messages-services",
    "docs/api/启动与节点参数.md": "reference/launch-parameters",
    "docs/api/配置文件.md": "reference/configuration",
    "docs/api/传感器数据.md": "reference/sensor-data",
    "docs/api/坐标与模型约定.md": "reference/coordinates",
    "docs/api/插件接口.md": "reference/plugin-api",
    "docs/api/C++核心接口.md": "reference/core-api",
    "docs/api/plugins/README.md": "reference/plugins/index",
    "docs/concepts/运行与坐标.md": "concepts/runtime",
    "docs/development/插件开发.md": "development/plugins",
    "docs/development/文档维护.md": "development/documentation",
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def relative_link(path, page):
    return Path(os.path.relpath(path, page.parent)).as_posix()


def anchor_name(docname, fragment):
    return docname.replace("/", "--") + "--" + fragment


def label_headings(content, docname):
    """Use page-scoped labels so equal anchors on separate pages stay distinct."""
    explicit = set(re.findall(r'<a\s+id="([^"]+)"></a>', content))
    result, seen, headings = [], {}, []
    in_fence = False
    for line in content.splitlines():
        if line.startswith("```"):
            in_fence = not in_fence
        match = re.match(r"^#{1,6}\s+(.+?)\s*#*\s*$", line) if not in_fence else None
        if match:
            title = re.sub(r"\[([^]]+)\]\([^)]+\)", r"\1", match.group(1))
            slug = "".join(c for c in title.lower() if c.isalnum() or c in "_- ").replace(" ", "-")
            count = seen.get(slug, 0)
            seen[slug] = count + 1
            slug += f"-{count}" if count else ""
            if slug not in explicit:
                result.extend([f"({anchor_name(docname, slug)})=", ""])
            headings.append({"source_heading": match.group(1), "output_docname": docname,
                             "myst_label": anchor_name(docname, slug)})
        if not in_fence:
            line = re.sub(r'<a\s+id="([^"]+)"></a>',
                          lambda m: f"({anchor_name(docname, m.group(1))})=\n", line)
        result.append(line)
    return "\n".join(result) + "\n", headings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="new/empty build directory")
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    if output == ROOT or output in ROOT.parents or (output.exists() and any(output.iterdir())):
        parser.error("output must be a new or empty directory, not the source root or an ancestor")
    output.mkdir(parents=True, exist_ok=True)
    source = output / "source"
    source.mkdir()
    mapping = dict(SLUGS)
    for path in sorted((ROOT / "docs/api/plugins").glob("litesim3d_*.md")):
        mapping[path.relative_to(ROOT).as_posix()] = "reference/plugins/" + path.stem
    for path in sorted((ROOT / "examples").rglob("*.md")):
        rel = path.relative_to(ROOT).as_posix()
        slug = rel.removesuffix(".md")
        if not slug.isascii():
            slug = "examples/" + path.parent.relative_to(ROOT / "examples").as_posix() + "/lidar-profiles"
        mapping[rel] = slug.removesuffix("/README") + "/index" if slug.endswith("/README") else slug

    all_markdown = sorted({*ROOT.glob("README*.md"), *(ROOT / "docs").rglob("*.md"),
                           *(ROOT / "examples").rglob("*.md")})
    current_folders = ("docs/tutorials/", "docs/usage/", "docs/api/", "docs/concepts/",
                       "docs/development/")
    unmapped = [path.relative_to(ROOT).as_posix() for path in all_markdown
                if path.relative_to(ROOT).as_posix().startswith(current_folders)
                and path.relative_to(ROOT).as_posix() not in mapping]
    if unmapped:
        raise ValueError(f"current documentation lacks a site mapping: {unmapped}")
    before = {path.relative_to(ROOT).as_posix(): digest(path) for path in all_markdown}
    records = [{"path": rel, "sha256_before": sha, "selected": rel in mapping,
                "selection_reason": "current reader documentation" if rel in mapping
                else "historical report/design or repository landing page; preserved as linked source",
                "docname": mapping.get(rel)} for rel, sha in before.items()]
    assets = {}
    copied = {}
    git = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, text=True, capture_output=True)
    revision = git.stdout.strip() if git.returncode == 0 else "source-copy"
    dirty = bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT)) if git.returncode == 0 else True
    version = (ROOT / "VERSION").read_text().strip()
    info = {"version": version, "revision": revision, "dirty": dirty,
            "label": f"{version} 开发文档 · {revision}" + (" · 本地修改" if dirty else ""),
            "chinese_search": importlib.util.find_spec("jieba") is not None}
    (source / "build-info.json").write_text(json.dumps(info, ensure_ascii=False))
    shutil.copyfile(ROOT / "sphinx-docs/conf.py", source / "conf.py")
    (source / "_static").mkdir()
    shutil.copyfile(ROOT / "sphinx-docs/docs.css", source / "_static/docs.css")
    trees = {"index": ["tutorials/index", "manual/index", "reference/index", "examples/index",
                        "versions", "concepts/runtime", "development/plugins", "development/documentation"],
             "tutorials/index": [], "manual/index": [], "reference/index": [], "reference/plugins/index": [],
             "examples/index": [], "examples/wheel_car_4w/index": [], "examples/drone_3d/index": []}
    for docname in mapping.values():
        if docname in trees["index"] or docname == "index":
            continue
        if docname == "reference/plugins/index":
            trees["reference/index"].append(docname)
            continue
        if docname.startswith("reference/plugins/"):
            trees["reference/plugins/index"].append(docname)
        elif docname.startswith("reference/"):
            trees["reference/index"].append(docname)
        elif docname.startswith("tutorials/"):
            trees["tutorials/index"].append(docname)
        elif docname.startswith("manual/"):
            trees["manual/index"].append(docname)
        elif docname.startswith("examples/"):
            parent = "examples/index"
            for family in ("wheel_car_4w", "drone_3d"):
                prefix = f"examples/{family}/"
                if docname.startswith(prefix) and docname != prefix + "index":
                    parent = prefix + "index"
            trees[parent].append(docname)
        else:
            trees["index"].append(docname)
    # The plugin landing page belongs only under the API section.
    if "reference/plugins/index" not in trees["reference/index"]:
        trees["reference/index"].append("reference/plugins/index")
    for values in trees.values():
        if len(values) != len(set(values)):
            raise ValueError("duplicate toctree membership")

    for relative, docname in mapping.items():
        original = ROOT / relative
        target = source / (docname + ".md")
        target.parent.mkdir(parents=True, exist_ok=True)
        content = original.read_text()
        # Hide source metadata only in prose, preserving XML comments in examples.
        parts = re.split(r"(^```[^\n]*\n.*?^```[ \t]*$)", content, flags=re.M | re.S)
        for index in range(0, len(parts), 2):
            parts[index] = re.sub(r'<!--.*?-->', '', parts[index], flags=re.S)
        content = "".join(parts)
        content, headings = label_headings(content, docname)
        next(record for record in records if record["path"] == relative)["headings"] = headings

        def rewrite(match):
            label, url = match.group(1), match.group(2)
            if re.match(r"\w+://", url) or url.startswith("mailto:"):
                return match.group(0)
            local, _, fragment = unquote(url.strip("<>")).partition("#")
            path = (original.parent / local).resolve() if local else original
            if not path.is_relative_to(ROOT) or not path.exists():
                raise ValueError(f"{relative}: unavailable link {url}")
            rel = path.relative_to(ROOT).as_posix()
            if rel in mapping:
                result = relative_link(source / (mapping[rel] + ".md"), target)
                if fragment:
                    reference = anchor_name(mapping[rel], fragment)
                    return "{ref}`" + label.replace("`", "") + " <" + reference + ">`"
            elif path.is_dir():
                result = "https://github.com/chiway-luo/litesim3d_ros2/tree/humble/" + quote(rel)
            else:
                if path not in copied:
                    sha = digest(path)
                    safe_name = path.name if path.name.isascii() else "resource" + path.suffix
                    folder = "_static/content" if path.suffix.lower() in {".svg", ".png", ".jpg", ".jpeg", ".gif"} else "_downloads"
                    dest = source / folder / sha[:12] / safe_name
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(path, dest)
                    copied[path] = dest
                    assets[rel] = {"destination": dest.relative_to(output).as_posix(), "sha256": sha}
                result = relative_link(copied[path], target)
                if path.suffix.lower() not in {".svg", ".png", ".jpg", ".jpeg", ".gif"}:
                    return "{download}`" + label.replace("`", "") + " <" + result + ">`"
            return f"[{label}]({result})"

        # Keep examples exactly as written. Rewrite prose links only.
        parts = re.split(r"(^```[^\n]*\n.*?^```[ \t]*$)", content, flags=re.M | re.S)
        for index in range(0, len(parts), 2):
            parts[index] = re.sub(r"\[([^]]*)\]\(([^)]+)\)", rewrite, parts[index])
        content = "".join(parts)
        if docname == "index":
            content += "\n\n当前快照：" + info["label"] + "。\n"
            if not info["chinese_search"]:
                content += "\n本次预览尚未启用中文分词，请使用 `IMU`、`QoS`、`rate_hz` 等关键词搜索。\n"
        if docname in trees and trees[docname]:
            content += "\n\n```{toctree}\n:hidden:\n:maxdepth: 2\n\n"
            content += "\n".join("/" + item for item in trees[docname]) + "\n```\n"
        target.write_text(content)

    if any(digest(ROOT / rel) != sha for rel, sha in before.items()):
        raise RuntimeError("source Markdown changed during conversion")
    state = output / ".sphinx-converter"
    state.mkdir()
    manifest = {"schema_version": 1, "mode": "create/migrate",
                "source_root": os.path.relpath(ROOT, output), "destination_root": ".",
                "sources": records, "assets": assets, "canonical_parents": trees,
                "generated_files": [{"path": path.relative_to(output).as_posix(), "sha256_generated": digest(path)}
                                    for path in sorted(source.rglob("*")) if path.is_file()]}
    (state / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    command = [sys.executable, "-P", "-m", "sphinx", "-E", "-a", "-n", "-W", "-b", "html",
               "-d", str(output / "doctrees"), "-w", str(output / "warnings.log"),
               str(source), str(output / "html")]
    built = subprocess.run(command, env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
    unchanged = all(digest(ROOT / rel) == sha for rel, sha in before.items())
    summary = {**info, "pages": len(mapping), "source_unchanged": unchanged,
               "returncode": built.returncode, "python": sys.executable,
               "scope": "Strict HTML build; browser/search and ROS tutorial validation are separate."}
    (output / "build-summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    return built.returncode or int(not unchanged)


if __name__ == "__main__":
    raise SystemExit(main())
