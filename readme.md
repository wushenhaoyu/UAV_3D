# MID360
`cd ros_libraries_ws`
`cd src`
`cd livox_ros_driver2`
`./build.sh ROS1`

注意以下用户名均为`flmg`，请根据需要修改 </br>

给权限 `sudo chown -R $USER:$USER /home/flmg/UAV_3D/build/` </br>

给权限 `sudo chown -R $USER:$USER /home/flmg/UAV_3D/devel/` </br>

给自启动脚本权限 `chmod +x /home/flmg/UAV_3D/run.bash` </br>

编写自启动文件 `sudo vim /etc/systemd/system/launch_tutorial.service` </br>
```
[Unit]
Description=ROS Launch Script
After=network.target

[Service]
ExecStart=/home/flmg/UAV_3D/run.bash
WorkingDirectory=/home/flmg/UAV_3D
User=flmg
Group=flmg
Restart=always

[Install]
WantedBy=multi-user.target
```
</br>

 重新载入配置 `sudo systemctl daemon-reload` </br>

 启用服务 `sudo systemctl enable launch_tutorial.service` </br>
 
 启动服务 `sudo systemctl start launch_tutorial.service` </br>
 
 检查服务状态 `sudo systemctl status launch_tutorial.service`</br>
 
 停止服务 `sudo systemctl stop launch_tutorial.service`</br>
 
 禁用服务 `sudo systemctl disable launch_tutorial.service`</br>

 喜欢的话请给个star，链接:<link href="https://github.com/wushenhaoyu/UAV_3D"> 
