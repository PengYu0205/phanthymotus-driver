# joints 独立分支

- 分支：`feat/q1-joints`。
- 工作目录：`/Users/4paradigm/phanthymotus/driver-joints`。
- 基线：`16e578f7d2fb2a8ac3892e22b0b58f397f90a798`，即 PR #584 的 Q1 电池驱动。
- 此目录为同一 Git 仓库的独立 worktree，已有组合分支 `feat/q1-joints-expression` 及其未提交改动保持原样。

## 本分支内容

新增 joints.py、q1_v3.urdf、tests/test_joints.py；最小修改 main.py、driver.yaml、Dockerfile、deploy/q1.py，补充文档。仅添加 joints/model；保留基线 battery。

不包含 expression、其他开发者插件、已撤下的运动卡或 Core 前端改动。Core 配套补丁位置为 `/Users/4paradigm/phanthymotus/core-changes/`。

提交目标为 `PengYu0205/phanthymotus-driver` 的 `feat/q1-joints`。本分支仅整理源码与配置，不替换机器人上的共享 Driver；不包含 Core 前端补丁。PR 另行创建。

6 项契约测试通过；尚需 live 数据下核对肘角、确认模型再分发许可及 ARM64 镜像构建。以草稿状态提交更准确。
