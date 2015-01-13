rhd
===

BSC RHD copy

Setup
=====

/sbin/route add default gw 192.168.7.1
echo "nameserver 8.8.8.8" > /etc/resolv.conf

git clone https://github.com/savnik/rhd
cd rhd/
./setup 

