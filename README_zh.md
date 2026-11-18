# Forge Controls

[English](README.md) | [简体中文](README_zh.md)

为 Forge/Dora 机器人数据流提供可复用的运动执行控制器。工作区包含 `packages/gripper_action_controller`（单关节位置夹爪控制器）和 `packages/joint_trajectory_controller`（固定关节组的位置轨迹控制器）；接收 Forge action 和新鲜 JointState，输出位置 JointCommand。控制核心与 Dora 传输适配分离，包含单元测试和 fake-plant 测试。低层实时伺服由硬件固件负责。

## 独立程序

使用 Python 3.12 执行 `scripts/build_pyinstaller.sh`，生成 `dist/gripper_action_controller`、`dist/joint_trajectory_controller` 和 `dist/fake_joint_plant`。构建验证帮助命令以及缺失或错误必需参数返回 argparse 状态码 2。

## 许可证

[Apache-2.0](LICENSE)。依赖保留原许可证，详见 [NOTICE](NOTICE) 和 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。发布独立程序前检查完整捆绑产物。

## 分版本发布

**发布规则（强制）：** 合并版本/锁文件修改 → 在已审查提交上创建 annotated 版本标签 → 从该标签精确构建 → 校验产物 → 发布。禁止使用移动分支、未打标签的提交构建 Release，禁止覆盖已发布标签。

| 维护线 | 源码 | 节点/包版本 | 运行时 |
| --- | --- | --- | --- |
| Forge 1.x | `stable/1.x` | `1.x`，标签 `v1.X.Y` | Forge 1.x / Dora 0.4.x |
| Forge 2.x | `main` | `2.x`，标签 `v2.X.Y` | Forge 2.x / Dora 1.x |

**旧版版本号统一：** `stable/1.x` 原为 0.x 的节点以 **1.0.0** 发布，保留 Forge 1.x / Dora 0.4.x 依赖与运行行为。已有 1.x 版本保留。节点/包主版本与 Forge 兼容线一致，不再接受 0.x 发布标签。

两条线均从 `main` 运行 **Build versioned release**。先将版本和锁文件修改通过 PR 合并至对应源码分支，再在已审查提交上创建未占用的 **annotated** 标签，标签必须匹配包元数据。禁止移动已有标签或复用已发布版本。工作流使用自身已审查的发布工具与精确标签源码，旧标签无需包含新发布脚本。

```bash
# 替换为已存在、已审查、未发布且与源码元数据一致的标签。
TAG=v1.0.0
gh workflow run release.yml --ref main -f release_line=1.x -f tag="$TAG"
TAG=v2.X.Y
gh workflow run release.yml --ref main -f release_line=2.x -f tag="$TAG"
```

流水线校验标签类型、源码提交、分支归属、源码版本及锁定的 Forge/Dora 兼容性；构建源码归档及项目支持的 wheel/sdist，检查 Python 分发元数据与 SHA-256，并创建或附加至 **GitHub Release 草稿**。不覆盖已发布 Release 或产物；发布前检查全部构建结果与验收依据。该流程不发布节点包至 PyPI。

设置 `build_binaries=true` 复用已有 Ubuntu 20.04 二进制工作流，保留原架构、CLI 和归档校验。也可直接运行 `build-ubuntu20.yml`，传入 `ref`、`version`、`release_line`；`ref` 必须为 annotated 标签，不能为移动分支。仅在所有选定任务通过后附加二进制产物。

源码归档保留标签中的文件；外部 SDK/submodule、模型下载与现场配置仍是独立前置条件。依赖或 SDK 构建失败不能视为发布验收通过。构建成功不代表单节点或串联 `dora run`、真机、真实模型、长稳态或回滚验收通过；每条验收链必须包含相同兼容线的 `adapter-forge-gateway`。Forge 2.x 属于 **breaking change**，整个部署的 Forge 库、节点版本与 Dora 必须对齐；1.x 保留在维护分支。
