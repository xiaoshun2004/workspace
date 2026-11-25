# CLAUDE.md

本目录为**乐白（lebai）机械臂**开发项目，使用 Python（`pylebai`）控制机械臂与夹爪。

## 权威参考手册

完整 SDK 文档已合并为单一 Markdown 文件，**开发时应以此为准**：

- 路径：`docs/lebai-sdk-manual.md`（202KB，74 章）
- 来源：https://lebai-robotics.github.io/lebai-sdk/ （Doxygen 生成，v2.0.4）

内容包含：Python/Java/.NET 应用指南、`Robot`/`Gripper`/`LuaRobot`/`Discovery` 等类参考、
机械臂运动/IO/状态/存储等 API 模块、以及 `*_data` 数据结构定义。

### 使用方式

- 查 API 签名：`grep -n "类名\|方法名" docs/lebai-sdk-manual.md` 或直接检索
- 查数据结构字段：搜索 `# lebai::l_master::XxxData Struct Reference` 章节
- 中文注释与函数参数表在手册中均为原文

### 重新生成（站点更新后）

```bash
cd lebai-sdk-docs            # 该目录被 .gitignore 排除, 仅本地使用
python3 build_manual.py mirror ../docs/lebai-sdk-manual.md
python3 build_pdf.py ../docs/lebai-sdk-manual.md ../docs/lebai-sdk-manual.pdf
```
