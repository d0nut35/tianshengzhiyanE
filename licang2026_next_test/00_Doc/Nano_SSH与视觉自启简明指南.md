# Nano SSH与视觉自启简明指南

## 1. 在Nano开启SSH

在Nano本机终端执行：

```bash
sudo systemctl enable --now ssh
systemctl status ssh --no-pager
hostname -I
```

`ssh`显示`active (running)`即正常。`hostname -I`可能显示多个地址，`172.17.0.1`通常是Docker地址，不用于电脑连接。Nano更换网络后IP可能变化。

## 2. Windows连接Nano

电脑和Nano需要处于能够互相访问的网络中。在Windows PowerShell执行：

```powershell
ssh donut@Nano的IP
```

例如：

```powershell
ssh donut@10.33.75.17
```

第一次连接输入`yes`，然后输入Nano用户`donut`的登录密码。输入密码时屏幕不显示字符。出现下面的提示符表示连接成功：

```text
donut@donut-desktop:~$
```

断开SSH：

```bash
exit
```

SSH卡死时先按回车，再输入`~.`强制断开。

## 3. 手动运行视觉程序

进入视觉工程：

```bash
cd /home/donut/programfile/licang/licang2026_-cv
```

检查脚本、摄像头和USB串口：

```bash
ls -l nano_uart_vision_responder.py
ls -l /dev/video0
ls -l /dev/serial/by-id/
```

正式联调使用自动场景和自动模式，一整行命令如下：

```bash
sudo python3 nano_uart_vision_responder.py --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --source /dev/video0 --scene auto --mode auto --response-delay-ms 0
```

按`Ctrl+C`停止。直接在SSH前台运行时，断开SSH通常也会结束程序；需要持续运行时使用下面的systemd服务。

## 4. 创建视觉开机自启服务

不使用文本编辑器，直接执行：

```bash
cat <<'EOF' | sudo tee /etc/systemd/system/nano-vision.service >/dev/null
[Unit]
Description=Nano UART vision responder
After=systemd-udevd.service

[Service]
Type=simple
WorkingDirectory=/home/donut/programfile/licang/licang2026_-cv
ExecStart=/usr/bin/python3 /home/donut/programfile/licang/licang2026_-cv/nano_uart_vision_responder.py --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --source /dev/video0 --scene auto --mode auto --response-delay-ms 0
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF
```

加载服务并立即启动，同时设置开机自启：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now nano-vision.service
```

## 5. 日常控制

查看状态：

```bash
systemctl status nano-vision.service --no-pager
```

正常状态应包含：

```text
Active: active (running)
```

查看最近日志：

```bash
journalctl -u nano-vision.service -n 100 --no-pager
```

实时查看日志：

```bash
journalctl -u nano-vision.service -f
```

重启视觉程序：

```bash
sudo systemctl restart nano-vision.service
```

暂时停止，下一次开机仍会自启：

```bash
sudo systemctl stop nano-vision.service
```

临时启动但不改变开机自启设置：

```bash
sudo systemctl start nano-vision.service
```

## 6. 关闭或恢复开机自启

关闭开机自启并立即停止视觉程序：

```bash
sudo systemctl disable --now nano-vision.service
```

重新开启开机自启并立即运行：

```bash
sudo systemctl enable --now nano-vision.service
```

只检查是否已设置开机自启：

```bash
systemctl is-enabled nano-vision.service
```

输出`enabled`表示已启用，输出`disabled`表示已关闭。

## 7. 修改服务命令后生效

修改`/etc/systemd/system/nano-vision.service`后执行：

```bash
sudo systemctl daemon-reload
sudo systemctl restart nano-vision.service
systemctl status nano-vision.service --no-pager
```

如果启动失败，查看原因：

```bash
journalctl -u nano-vision.service -n 100 --no-pager
```

常见原因是摄像头不是`/dev/video0`，或USB串口在`/dev/serial/by-id/`中的名称发生变化。

