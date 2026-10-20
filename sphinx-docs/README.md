# LiteSim 3D 文档站

此项目从仓库现有 Markdown 生成站点副本。维护正文仍编辑 `docs/`；`source/` 和 HTML 只存在于指定的构建目录中。侧栏按教程、操作手册、接口、原理与开发、车型组织，历史报告以原始文件链接保留。

## 构建

使用 Python 3.12 和 [requirements.txt](requirements.txt) 声明的依赖；中文全文搜索另需 jieba 0.42.1。构建器不会安装依赖或修改系统。

从仓库根目录运行，输出路径必须尚不存在或为空：

```bash
python3.12 -P sphinx-docs/tools/build.py --output /tmp/litesim-docs-preview
```

构建器会执行 Sphinx 的严格模式（`-E -a -n -W`），保存 `source/`、`html/`、`warnings.log`、`build-summary.json` 与 `.sphinx-converter/manifest.json`。源 Markdown 的转换前后哈希必须一致。重复构建使用新的输出目录，避免覆盖手工修改或混入旧 HTML。

预览：

```bash
python3.12 -P -m http.server 8000 --bind 127.0.0.1 --directory /tmp/litesim-docs-preview/html
```

打开 `http://127.0.0.1:8000`，检查完成后 Ctrl+C 关闭服务器。所有必要样式、图示与站点脚本均在本地；访问源码外链仍需网络。

## 搜索与版本

没有 jieba 时，只启用英文/API 关键词搜索，构建摘要标记 `chinese_search: false`。不能把生成了 searchindex.js 当作中文搜索通过；安装分词依赖后仍需在浏览器搜索实际正文中的中文术语。

页眉显示 VERSION、基础提交及本地修改状态。当前没有已冻结的发布文档集合，因此不提供空的版本切换菜单。正式发布时从对应标签构建独立目录，再把真实存在的版本接入导航；不要覆盖旧快照。

## 页面和文件

每个正文页面在侧栏有一个父级。站点使用稳定 ASCII 路径，原 Markdown 和显式锚点保留在仓库中。源码文件、示例 YAML/Xacro、历史报告作为下载链接复制；目录外链指向 GitHub。生成清单记录所有选取/排除的 Markdown、正文映射、文件哈希和资源来源。

转换不运行教程代码；教程运行检查与站点构建分别报告。CI 的文档站任务只生成并上传预览产物，不发布网站。
