#!/usr/bin/env python3
"""Render plugin reference tables from the shipped, contract-tested YAML files."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[2]
DESTINATION = ROOT / "docs/api/plugins"


def comment_before(lines, line):
    comments = []
    for text in reversed(lines[:line]):
        text = text.strip()
        if not text.startswith("#"):
            break
        comments.append(text[1:].strip())
    return " ".join(reversed(comments))


def value_type(node, name):
    if isinstance(node, yaml.MappingNode):
        return "object"
    if isinstance(node, yaml.SequenceNode):
        if name == "wheels":
            return "object[]"
        if name.endswith("joints") or name == "excluded_links":
            return "string[]"
        return "number[]"
    tag = node.tag.rsplit(":", 1)[-1]
    if tag in ("int", "float"):
        return "integer" if name in ("width", "height", "columns") else "number"
    return {"str": "string", "bool": "boolean"}.get(tag, tag)


def fields(node, lines, prefix=""):
    for key, value in node.value:
        name = f"{prefix}{key.value}"
        description = comment_before(lines, key.start_mark.line)
        if not description:
            raise ValueError(f"{name}: missing parameter explanation")
        required = description.startswith("必填")
        # Sample geometry must never be presented as an implementation default.
        if required:
            default = "无；必填"
        elif isinstance(value, yaml.MappingNode):
            default = "见子字段"
        else:
            parsed = yaml.safe_load(yaml.serialize(value))
            default = "`" + json.dumps(parsed, ensure_ascii=False, separators=(",", ":")) + "`"
        yield (name, value_type(value, key.value), default, description)
        if isinstance(value, yaml.MappingNode):
            yield from fields(value, lines, name + ".")
        elif key.value == "wheels" and isinstance(value, yaml.SequenceNode):
            if not value.value:
                raise ValueError("wheels must contain a complete example")
            yield from fields(value.value[0], lines, name + "[].")


def generated(root=ROOT):
    result = {}
    index = [
        "# 标准插件参数", "",
        "按实例选择插件，参数在启动时读取；修改后重新启动，不支持运行中热切换。",
        "数值默认值来自随包完整参数文件，并由插件契约测试与实现的 `defaults()` 校对。"
        "标为必填的轮几何、关节、link 和 topic 没有缺省值，参数文件中的具体值只是示例。",
        "覆盖顺序、类生命周期和异常语义见[插件接口](../插件接口.md)。", "",
        "| 实现包 | 类别 | 参数参考 |", "| --- | --- | --- |",
    ]
    for manifest in sorted((root / "plugins").rglob("package.xml")):
        package = manifest.parent
        source = package / "config/parameters.yaml"
        if not source.is_file():
            continue
        xml = ET.parse(package / "plugins.xml")
        classes = xml.findall(".//class")
        if len(classes) != 1:
            raise ValueError(f"{package.name}: expected one documented implementation")
        cls = classes[0]
        category = cls.attrib["base_class_type"].rsplit("::", 1)[-1]
        code = (package / "src/plugin.cpp").read_text()
        schema_code = code.split("void initialize", 1)[0]
        dependencies = [source, package / "src/plugin.cpp", package / "plugins.xml"]
        if "sensorDefaults(" in schema_code:
            defaults_source = root / "litesim3d_core/src/sensor_plugin.cpp"
            schema_code = defaults_source.read_text().split("Json sensorDefaults", 1)[1].split('if (kind', 1)[0]
        else:
            defaults_source = root / "litesim3d_plugin_api/src/motion.cpp"
        dependencies.append(defaults_source)
        required = set(re.findall(r'\{\s*"([^"]+)"\s*,\s*nullptr\s*\}', schema_code))
        content = source.read_text()
        node = yaml.compose(content)
        rows = list(fields(node, content.splitlines()))
        documented_required = {name for name, _, default, _ in rows if "." not in name and default == "无；必填"}
        if required != documented_required:
            raise ValueError(f"{package.name}: required field comments differ from defaults(): "
                             f"source={sorted(required)}, YAML={sorted(documented_required)}")
        fingerprint = hashlib.sha256(b"".join(p.read_bytes() for p in dependencies)).hexdigest()
        page = [
            f"# {package.name}", "", f"插件类：`{cls.attrib['name']}`；类别：`{category}`。", "",
            "本表所有参数在启动时读取。`number` 接受有限实数；数组长度、单位、范围和关联条件见说明列。",
            "必填项没有默认值；请在机器人配置中提供实际值。可选项为实现缺省值，车型配置可以覆盖。", "",
            f"来源：[parameters.yaml](../../../{package.relative_to(root)}/config/parameters.yaml)、"
            f"[实现](../../../{package.relative_to(root)}/src/plugin.cpp)。", "",
            "| 字段 | 类型 | 默认值 | 单位、范围与含义 |",
            "| --- | --- | --- | --- |",
        ]
        for name, kind, default, description in rows:
            page.append(f"| `{name}` | {kind} | {default} | {description.replace('|', '&#124;')} |")
        page += ["", "未知字段、错误类型、非法数值或缺失必填项会在加载阶段报错。"
                 "含禁用传感器的配置仍须满足参数与挂载要求。"
                 "坐标/关节条件见[坐标与模型约定](../坐标与模型约定.md)。",
                 "", "[返回插件参数目录](README.md) · [覆盖和加载规则](../插件接口.md)", "",
                 f"<!-- generated by scripts/docs/generate_plugin_reference.py; source-sha256: {fingerprint} -->", ""]
        result[Path("docs/api/plugins") / f"{package.name}.md"] = "\n".join(page)
        index.append(f"| `{package.name}` | `{category}` | [字段表]({package.name}.md) |")
    index += ["", "维护时修改实现包的参数文件与代码，再运行 "
              "`python3 scripts/docs/generate_plugin_reference.py --write`。"
              "静态检查拒绝过期表格；运行期默认值由安装后的插件契约测试验证。", ""]
    result[Path("docs/api/plugins/README.md")] = "\n".join(index)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true", help="update generated Markdown")
    args = parser.parse_args()
    failed = []
    try:
        outputs = generated()
        for relative, content in outputs.items():
            path = ROOT / relative
            if args.write:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content)
            elif not path.is_file() or path.read_text() != content:
                failed.append(str(relative))
    except (ValueError, KeyError, yaml.YAMLError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    print(json.dumps({"status": "FAIL" if failed else "PASS", "pages": len(outputs),
                      "outdated": failed, "mode": "write" if args.write else "check"}))
    return int(bool(failed))


if __name__ == "__main__":
    raise SystemExit(main())
