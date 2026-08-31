# 终端 08B：GraspNet RGB-D 推理服务

只在路线 B 使用。运行 `bash 运行.sh`。节点缓存同步 RGB-D，并提供 `/ur3_graspnet6d/infer` 服务；只有规划器调用服务时才执行一次网络推理。

首次加载 checkpoint 需要时间。成功后应显示 GraspNet ready，并持续收到同步 RGB-D。

