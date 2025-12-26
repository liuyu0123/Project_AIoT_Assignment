@echo off
:: 1. 端口代理（把局域网 33210 转到本地 127.0.0.1:33210）
netsh interface portproxy reset
netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=33210 connectaddress=127.0.0.1 connectport=33210

:: 2. 防火墙放行
netsh advfirewall firewall delete rule name="红海Pro-HTTP-热点"
netsh advfirewall firewall add rule name="红海Pro-HTTP-热点" dir=in action=allow protocol=TCP localport=33210 remoteip=192.168.137.0/24

echo 代理共享规则已恢复，任意键退出
pause