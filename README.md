# pka2026_sentry_nav（包含决策部分串口部分）

这是福建师范大学pikachu战队2026赛季导航，参考了北极熊战队的导航算法。

## 一、日志（继承25赛季）

(已更新到哨兵pc上！！！)
20250313：已更新适配联盟赛串口

20250313：由于 NAV2 humble 发行版出于避免破坏原有接口的原因，依然使用 Twist 类型（不含时间戳），humble 往后的版本才使用 TwistStamped，导致无法直接实现 cmd_vel 与 odometry 的时间戳对齐。因此，本功能包暂时订阅 local_plan 话题（由局部路径规划器发布），以获取时间戳，将它的时间戳视为 cmd_vel 的时间戳，以间接实现时间戳对齐。所以把北极熊最新的fake_transform移植了进来。

20250314：添加心跳节点!

20251228: 修改了cmakelist.txt

## 二、自启动

- 编译程序后，进入rm_upstart文件夹

```bash
cd rm_upstart
```

- 修改**rm_watch_dog.sh**中的`NAMESPACE`（ros命名空间）、`NODE_NAMES`（需要看门狗监控的节点）和`WORKING_DIR` （代码路径）

- 注册服务

```bash
sudo chmod +x ./register_service.sh
sudo ./register_service.sh

# 正常时有如下输出
# Creating systemd service file at /etc/systemd/system/rm.service...
# Reloading systemd daemon...
# Enabling service rm.service...
# Starting service rm.service...
# Service rm.service has been registered and started.
```

- 查看程序状态

```bash
systemctl status rm
```

- 查看终端输出

```
查看screen.output或~/fyt2024-log下的日志
```

- 关闭程序

```bash
systemctl stop rm
```

- 取消自启动

```bash
systemctl disable rm
```

## 
