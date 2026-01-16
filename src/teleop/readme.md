# 为 televuer 模块配置 SSL 证书，以便 XR 设备（如 Pico / Quest / Apple Vision Pro）通过 HTTPS / WebRTC 安全连接
# 1. 生成证书文件
# 1.1 如果您使用 pico / quest 等 xr 设备
(tv) unitree@Host:~/xr_teleoperate/teleop/televuer$ openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout key.pem -out cert.pem
# 1.2 如果您使用 apple vision pro 设备
(tv) unitree@Host:~/xr_teleoperate/teleop/televuer$ openssl genrsa -out rootCA.key 2048
(tv) unitree@Host:~/xr_teleoperate/teleop/televuer$ openssl req -x509 -new -nodes -key rootCA.key -sha256 -days 365 -out rootCA.pem -subj "/CN=xr-teleoperate"
(tv) unitree@Host:~/xr_teleoperate/teleop/televuer$ openssl genrsa -out key.pem 2048
(tv) unitree@Host:~/xr_teleoperate/teleop/televuer$ openssl req -new -key key.pem -out server.csr -subj "/CN=localhost"

## 创建 server_ext.cnf 文件，输入以下内容（IP.2 地址应与您的 主机 IP 地址匹配，假设此处地址为 192.168.123.2。可以使用 `ifconfig` 等类似命令查询）
(tv) unitree@Host:~/xr_teleoperate/teleop/televuer$ vim server_ext.cnf
subjectAltName = @alt_names
[alt_names]
DNS.1 = localhost
IP.1 = 192.168.123.164
IP.2 = 192.168.123.2

(tv) unitree@Host:~/xr_teleoperate/teleop/televuer$ openssl x509 -req -in server.csr -CA rootCA.pem -CAkey rootCA.key -CAcreateserial -out cert.pem -days 365 -sha256 -extfile server_ext.cnf

(tv) unitree@Host:~/xr_teleoperate/teleop/televuer$ ls
build  cert.pem  key.pem  LICENSE  pyproject.toml  README.md  rootCA.key  rootCA.pem  rootCA.srl  server.csr  server_ext.cnf  src  test

# 通过 AirDrop 将 rootCA.pem 复制到 Apple Vision Pro 并安装它

# 开启防火墙
(tv) unitree@Host:~/xr_teleoperate/teleop/televuer$ sudo ufw allow 8012

# 2. 配置证书路径，以下方式任选其一
# 2.1 用户配置目录（可选）
(tv) unitree@Host:~/xr_teleoperate/teleop/televuer$ mkdir -p ~/.config/xr_teleoperate/
(tv) unitree@Host:~/xr_teleoperate/teleop/televuer$ cp cert.pem key.pem ~/.config/xr_teleoperate/
# 2.2 环境变量配置（可选）
(tv) unitree@Host:~/xr_teleoperate/teleop/televuer$ echo 'export XR_TELEOP_CERT="$HOME/xr_teleoperate/teleop/televuer/cert.pem"' >> ~/.bashrc
(tv) unitree@Host:~/xr_teleoperate/teleop/televuer$ echo 'export XR_TELEOP_KEY="$HOME/xr_teleoperate/teleop/televuer/key.pem"' >> ~/.bashrc
(tv) unitree@Host:~/xr_teleoperate/teleop/televuer$ source ~/.bashrc
```
