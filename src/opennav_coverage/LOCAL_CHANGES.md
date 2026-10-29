# OpenNav Coverage 本地源码说明

本目录作为普通源码纳入工作区主仓库，由主仓库统一跟踪，不使用 Git 子模块。

## 来源与版本

- 上游项目：[open-navigation/opennav_coverage](https://github.com/open-navigation/opennav_coverage)。
- 导入前本地仓库 HEAD：`be1ac789b2930470f3c4922b29d17156a9412ce1`。
- 导入内容包含该本地版本及导入时尚未提交的修改；上述提交号不代表本目录的完整状态。
- 保留上游 `LICENSE`、文件版权声明和 `README.md`。

原仓库的 Git 元数据保存在导入机器的主仓库 `.git/nested-repo-backups/` 下，
用于本地恢复历史，不随主仓库提交。

## 随源码保存的本地适配

- 提供独立的 `swath_spacing` 参数，分离条带间距与实际作业宽度，并校验参数。
- 拼接转弯路径时保留原始位姿，避免重采样破坏转弯曲率。
- 按实际作业宽度显示条带，并向晚连接的 RViz 订阅者提供最新结果。
- 保留条带间距、前进转弯曲率和 RViz 显示的回归测试。
- 保留四个包中的 `COLCON_IGNORE` 文件，以维持当前工作区的构建范围。
- 保存现有 `启动命令.txt`；其中的设备路径和现场参数需按运行环境核对。

导入时工作树已移除 `opennav_coverage_demo`，本次保存当前状态，不恢复示例包。
Fields2Cover 仍作为外部依赖单独准备。

## 统一覆盖任务参数租约

- `task_parameter_owner` 在原子参数事务中绑定外层覆盖 goal UUID，匹配的
  `task_parameter_release` 才能清除；租约期间拒绝旁路参数改写和生命周期停用。
- 规划器保留实际配置，桥接只设置请求中的作业宽度/条带方向，并读回完整生效配置。
- 计算期间使用受同一互斥锁保护的边界校验参数缓存，避免与参数服务的锁顺序死锁。
- 空 `default_custom_order` 使用明确的整数数组类型，支持完整参数快照读回。
- 真实规划器租约与并发计算回归位于 `nav2_mission/test/test_coverage_parameter_lease_ros.py`。
