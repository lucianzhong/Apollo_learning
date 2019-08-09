1. Apollo官方:

git lfs clone git@gitee.com:mirrors/apolloauto.git -b r3.5.0
 
df -hl  #Ubuntu 查看磁盘空间大小命令


2. SSH public key

 解决办法通过 ssh-add

eval "$(ssh-agent -s)"

ssh-add


3. Github 加速 in Ubuntu & the same in Docker
   用浏览器访问 IPAddress.com 使用 IP Lookup 工具获得这个域名的ip地址，该网站可能需要梯子，输入上述域名后，分别获得github.com和github.global.ssl.fastly.net对应的ip，比如192.168.xx.xx和185.31.17.xx。准备工作做完之后，在vi打开的hosts文件中添加如下格式：

192.168.xx.xx github.com

185.31.17.xx github.global.ssl.fastly.net

      然后esc退出编辑模式，输入wq，保存hosts文件，修改hosts结束。此时，如果直接访问github可能不会立即生效，因为有DNS缓存，并没有按照最新的修改配置访问。

      最后，我们在命令行中输入sudo dscacheutil -flushcache，更新DNS缓存。

127.0.0.1	localhost
127.0.1.1	zy
10.2.35.202	docker.51vr.local
151.101.185.194	github.global.ssl.fastly.net
140.82.113.4	github.com

最暴力的方法刷dns，重启网络：

sudo /etc/init.d/networking restart

重启网络服务:
 /etc/init.d/networking restart

The folder contains source code

zy@zy:/home/apollo_offical/apollo_r.5.0/apolloauto$



1. 
   GPU version:
   sudo ./apollo.sh build_gpu  

   CPU version:
    ./apollo.sh build_cpu

   Easy version:
   bash apollo.sh build -j 8


The source code:
zy@zy:/home/apollo-r3.5.0$ 


apollo 3.5 official image:
 https://github.com/ApolloAuto/apollo/tree/r3.5.0
Apollo 3.5的构建方法:
https://blog.csdn.net/davidhopper/article/details/85097502

# Start the docker.
bash docker/scripts/dev_start.sh
# Step into the docker.
bash docker/scripts/dev_into.sh
# Build the apollo project in the docker.
# -j 8 depends on the number of CPU cores on your machine.
bash apollo.sh build -j 8

bash scripts/bootstrap.sh   
http://localhost:8888/


Restart Dreamview:
  bash scripts/bootstrap.sh stop
  bash scripts/bootstrap.sh start


Dreamview  用gdb调试:
$ gdb --args /apollo/bazel-bin/modules/dreamview/dreamview --flagfile=/apollo/modules/dreamview/conf/dreamview.conf
