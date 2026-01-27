# net_off.sh
sudo sysctl -w net.ipv4.ip_forward=0
sudo iptables -t nat -F
