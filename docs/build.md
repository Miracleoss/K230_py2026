# 编译与烧录

## 下位机（STM32F405 / Keil MDK-ARM）

```powershell
# 用 Keil 打开对应版本的工程
firmware\v33\MDK-ARM\A133P_Dart.uvprojx   # 硬件 V33
firmware\v34\MDK-ARM\A133P_Dart.uvprojx   # 硬件 V34
```

- Target 名：`A133P_Dart`
- 输出：`MDK-ARM/A133P_Dart/A133P_Dart.axf`、`.hex`
- 这些输出文件、`.o`、`.map`、`JLinkLog.txt`、`*.uvguix.*` 都已被 `.gitignore` 排除，**不要强行 add**。
  如果你发现 `git status` 里冒出这些文件，说明 `.gitignore` 被破坏或文件被 `-f` 强行加进去了。
- 工程里带 `JLinkLog.txt`，说明用的是 J-Link。换机器后需要自己重配 J-Link 的 flash download 设置（这部分是本地状态，不进库）。

### 关于 CubeMX 重生成

`Core/` 和 `Drivers/` 是 `A133P_Dart.ioc` 的生成物。**重新 Generate 会覆盖 `Core/`**：

- 生成前先提交一次，生成后用 `git diff` 看清被改了哪些，只保留确实需要的那部分。
- CubeMX 的生成配置里要确认 `User/` 目录没有被勾进生成范围，否则自己写的层会被冲掉。
- 两个版本的 `.ioc` 同名，别在同一次 CubeMX 会话里来回切目录，容易生成错版本。

## 上位机（A133 / Linux）

`host/a133/` 只放**你们自己写的应用层**（视觉、制导解算、串口对接）。

**A133 的 BSP / 内核 / buildroot 一律不进这个仓库。** Allwinner 的 SDK 动辄数 GB，GitHub 单文件超 100 MB 会直接拒收，仓库会被撑爆且无法回收。正确做法：

- 用 `repo` manifest 或 git submodule，在主仓库里只提交 manifest 文件；
- 或者写一个 `scripts/fetch-sdk.sh`，把 SDK 的获取方式和版本号固定下来；
- 在 `docs/build.md`（本文件）里写清楚 SDK 的版本和来源，别人照着一句话就能拉下来。

这一条是硬约束。已经进库的大文件一旦被 push，只能靠改写历史 + 强推才能清掉，代价很大。

## 新机器最快上手路径

```powershell
git clone git@github.com:Miracleoss/K230_py2026.git guided-dart   # 改名后用新名
cd guided-dart
git branch -a          # 只会看到 main 和 archive/*，archive/* 是历史，别在此工作
```

下位机：装 Keil MDK + 对应器件包 → 打开 `firmware/vXX/MDK-ARM/A133P_Dart.uvprojx` → 编译 → 配 J-Link 烧录。
上位机：按上面的 SDK 说明拉取 SDK → 把 `host/a133/` 里的代码放进去编译。
