# 制导飞镖 · Guided Dart

上位机 + 下位机联合仓库。两套硬件、两条代码线，共用一条 `main`。

| 分线 | 硬件 | 位置 |
| --- | --- | --- |
| 下位机 | STM32F405RGT6 @168 MHz | `firmware/` |
| 上位机 | Allwinner A133（Linux） | `host/` |

## 目录

```
guided-dart/
├─ firmware/
│  ├─ v33/          # 下位机 · 硬件 V33（仍在使用）
│  ├─ v34/          # 下位机 · 硬件 V34（新一代）
│  └─ common/       # 两版共享层（第二期抽取，目前为空）
├─ host/
│  └─ a133/         # 上位机应用层；A133 SDK 不进库
├─ docs/
│  ├─ protocol.md          # F405 ↔ A133 串口协议（唯一的跨端合同）
│  ├─ hardware-v33-v34.md   # 两块板的源码差异清单
│  └─ build.md             # 编译 / 烧录步骤
└─ .github/CODEOWNERS
```

## 上手

- **下位机**：Keil 打开 `firmware/v33/MDK-ARM/A133P_Dart.uvprojx`（或 `v34` 下同名文件），target 为 `A133P_Dart`，输出到 `MDK-ARM/A133P_Dart/`。
- **上位机**：见 `docs/build.md`。
- **改协议前**：先读 `docs/protocol.md`，跨端改动必须两个人一起过。

## 分支规范

- `main` 是唯一主线，任何时候都能编译通过。
- 不直接往 `main` 推；开 `feat/xxx`、`fix/xxx` 分支，走 PR，另一个人 review 后合并。
- 拉取用 `git pull --rebase`，不要制造无意义的 merge commit。
- 严禁各自在自己分支上堆一大坨长期不合并的提交（上一轮就是这样停掉的）。

## V33 / V34 的同步规则

两版是同一套分层架构，`User/` 下 8 个层目录完全对应。约定：

- **共享层**（改一处必须两边同步）：`Algorithm`、`Common`、`Device`、`Protocol/dart_light_protocol`、`Task`
- **板级层**（各自独立）：`Config`、`Driver`、`Application`、`Core`、`MDK-ARM`、`*.ioc`

分层的边界和逐文件差异见 `docs/hardware-v33-v34.md`。

## 历史归档

F103 + K230 那一代的代码完整保留在标签和归档分支里，**不要修改、不要删除**：

- 标签：`v0.1-f103-mcu`、`v0.1-k230-host`、`v0.1-host-dev`
- 分支：`archive/f103`、`archive/k230-host`、`archive/host-dev`

它们和现在的 `main` 没有共同祖先关系，这是有意的——旧工程是按另一套硬件长出来的，不做硬合并。
