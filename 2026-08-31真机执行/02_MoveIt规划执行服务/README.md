# 终端 02：MoveIt 规划与执行服务

运行 `bash 运行.sh`。它启动 `move_group`、OMPL、碰撞检测和右臂轨迹执行接口。`allow_trajectory_execution:=true` 只表示 MoveIt 具备执行能力，并不会自行运动。

成功标志是 `/move_group` 正常出现，且右臂控制器可被 MoveIt 识别。

