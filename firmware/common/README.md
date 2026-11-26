# firmware/common

两版共享层的抽取目标目录，**目前为空**，一期不往里放东西。

V33 这一季还要继续维护，所以第一阶段保持 `firmware/v33` 和 `firmware/v34` 各自完整、各自能独立编译——先把"两份代码都在库里、都能复现"这件事稳住，不做结构性重构。

等 V33 退役或两版差异稳定后，第二阶段再按 `docs/hardware-v33-v34.md` 的清单往下搬：

- 预期共享：`Algorithm`、`Common`、`Device`、`Task`、`Protocol/dart_light_protocol`
- 预期保留在各自版本：`Config`、`Driver`、`Application`、`Core`、`MDK-ARM`

搬的时候两个 `.uvprojx` 用 `..\common\` 相对路径引用同一批文件，让"两边不一致"在编译期就暴露，而不是靠人记。
