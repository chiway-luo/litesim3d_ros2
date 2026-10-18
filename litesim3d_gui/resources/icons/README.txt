LiteSim 3D Dock 矢量图标 · 2026-10-18

设计：蓝色圆角底板、白色三维立方体、青色激光扫描环。
底板之外保留透明背景；不含文字，方便小尺寸显示。

主文件：litesim3d.svg。全部由路径、几何图元和渐变构成，无嵌入位图、字体或外部资源。
Linux / Qt：直接使用 SVG，按目标尺寸和屏幕倍率渲染。
macOS 图标包：LiteSim3D.icns，包含标准与 Retina 表示。
PNG / ICNS 均从 SVG 分别渲染各尺寸，不放大已有的小位图。
兼容 PNG 尺寸：16、24、32、48、64、128、256、512、1024 像素。

Qt 窗口通过 ../app_icons.qrc 仅嵌入 SVG，main() 使用 QIcon 加载矢量源。
Qt 已启用 AA_UseHighDpiPixmaps，高清屏幕按实际像素倍率渲染图标。
资源随 viewer_native 编译，运行时无需访问源码目录。
CMake 安装时另将 SVG、PNG 和 ICNS 放入 share/litesim3d_gui/icons。
Ubuntu 构建依赖 libqt5svg5-dev；运行依赖 libqt5svg5（含 SVG 图标引擎）。
项目正常构建会将该矢量资源编入三维查看器，安装后启动即可使用。
本次没有创建 .app、注册系统启动器或修改 Dock 固定项。
Linux 桌面环境的启动器/Wayland 图标关联仍需对应 .desktop 配置。

维护：编辑 SVG 后，在提供 PyQt5.QtSvg 的环境中执行
  python3 litesim3d_gui/resources/export_icons.py
即可重新生成同目录的 PNG 与 ICNS。日常构建无需执行导出脚本。

验证：Qt 离屏 SVG 资源加载、16～4096 像素渲染及 2 倍 DPI 检查通过；
macOS 本地构建通过，ICNS 通过系统解码。两端实际 Dock 显示尚未实测。

参考：Qt QIcon / SVG 引擎
https://doc.qt.io/archives/qt-5.15/qicon.html
最初的位图概念提示词保留在 generation-prompt.txt；当前矢量稿为手工重绘。
