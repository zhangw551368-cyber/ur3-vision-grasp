# 终端 10B：冻结目标、完整规划、缓存执行

只在路线 B 使用，严格分为两个阶段。

## 阶段 A：只规划

从原始 1280×720 分类图确认每个物体的中心和半径，然后设置 `TARGET_SPECS`：

```bash
TARGET_SPECS='metal_flange:818:192:60,large_ring:786:112:50' bash 01_只规划并生成缓存.sh
```

上面是 2026-08-24 最终成功场景的历史示例，不是当前场景通用坐标。物体或相机有任何移动，都必须替换为当前像素。

该脚本会为全部目标完成推理、候选过滤、抓取、抬升、棋盘释放和返回规划，然后生成：

```text
runtime/multi_sequence_plan.json
runtime/cached_sequence_plan.pkl
```

规划阶段不发送运动命令。必须在 RViz 审查全部轨迹。

## 阶段 B：真机执行

1. 运行 `bash 02_检查真机状态.sh`。
2. 人工确认安全清单和当前缓存。
3. 将原配置 `config/right_arm_green_table.yaml` 中唯一的 `execution.enabled` 临时改为 `true`。
4. 运行 `bash 03_执行当前缓存.sh` 并输入 `EXECUTE`。
5. 执行结束立即把 `execution.enabled` 恢复为 `false`。

缓存默认最多保存 1800 秒；机器人当前关节与缓存起点误差不得超过 0.03 rad。执行期间不读取相机，夹爪未得到 `gOBJ=2` 时停止并恢复。

