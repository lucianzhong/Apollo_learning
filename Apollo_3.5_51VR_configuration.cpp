

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

127.0.0.1 localhost
127.0.1.1 zy
10.2.35.202 docker.51vr.local
151.101.185.194 github.global.ssl.fastly.net
140.82.113.4  github.com

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





// SimOne.exe
D:\Cybertron\UnrealEngine\Output\Development\WindowsNoEditor\SimOne\Binaries\Win64


1. Rancher 
	Image: docker.51vr.local:6000/apolloauto/apollo:dev5 

2. IN Ubuntu terminal: docker ps -a
		  	docker exec -it 1c10ae5c4777  bash 
        bash scripts/bootstrap.sh
        source /apollo/scripts/apolo_base.sh
        cyber_recorder play -f docs/demo_guide/demo_3.5.record --loop

        run.sh & kill.sh


The maps located in: root@apollo5-apollo-ubuntu14-1:/Cybertron2/apollo/modules/map/data

3. The GitLab

	http://git.51vr.local/51World/apollo/tree/a3.5/docker     



6. only CyberBridge bazel build:
  root@apollo5-apollo-ubuntu14-1:/apollo# bazel build //CyberBridge:libApolloCyber.so


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Apollo官方:

1. 
   GPU version:
   sudo ./apollo.sh build_gpu  

   CPU version:
    ./apollo.sh build_cpu

   Easy version:
   bash apollo.sh build -j 8


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


查看Bazel依赖图:
bazel query --nohost_deps --noimplicit_deps 'deps(//modules/dreamview:dreamview)'


2. 51VR

without perception
  apollo5-apollo-ubuntu14-1

 docker exec -it  1c10ae5c4777 bash 


root@apollo5-apollo-ubuntu14-1:/apollo# ./apollo.sh

Start the dreamview:
    bash scripts/bootstrap.sh 


3. 错误

./apollo.sh build_cpu


INFO: (07-23 01:17:44.831) Found 3742 targets...
ERROR: (07-23 01:17:46.748) /apollo/modules/perception/lidar/lib/segmentation/cnnseg/BUILD:84:1: Linking of rule '//modules/perception/lidar/lib/segmentation/cnnseg:cnn_segmentation_test' failed (Exit 1).
/usr/bin/ld: warning: libmkldnn.so.0, needed by bazel-out/local-dbg/bin/_solib_k8/_U@paddlepaddle_S_S_Cpaddlepaddle___Uexternal_Spaddlepaddle_Slib/libpaddle_fluid.so, not found (try using -rpath or -rpath-link)

cd 

need to mount the volume: paddlepaddle_volume-x86_64-1.0.0




4. only CyberBridge bazel build:
  root@apollo5-apollo-ubuntu14-1:/apollo# bazel build //CyberBridge:libApolloCyber.so




5.  // How to Run Perception Module on Your Local Computer

  Build Apollo:  ./apollo.sh build_opt_gpu

  查看显卡名称以及驱动版本:  nvidia-smi


  // install nvidia-docker  // https://www.jianshu.com/p/f25ccedb996e   //https://www.dongliwu.com/archives/61/
  # If you have nvidia-docker 1.0 installed: we need to remove it and all existing GPU containers
  docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f
  sudo apt-get purge -y nvidia-docker

  # Add the package repositories
  curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey |  sudo apt-key add -  distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
  curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list |     sudo tee /etc/apt/sources.list.d/nvidia-docker.list
  sudo apt-get update

  # Install nvidia-docker2 and reload the Docker daemon configuration
  sudo apt-get install -y nvidia-docker2
  sudo pkill -SIGHUP dockerd

  # Test nvidia-smi with the latest official CUDA image
  docker run --runtime=nvidia --rm nvidia/cuda:9.0-base nvidia-smi


  // 配置daemon的默认运行时
  在安装完成nvidia-docker2之后，nvidia-docker2已经默认在/etc/docker/daemon.json文件中写入了以下内容：

  {
      "default-runtime": "nvidia",
      "runtimes": {
          "nvidia": {
              "path": "/usr/bin/nvidia-container-runtime",
              "runtimeArgs": [],
              "registry-mirrors": ["https://gemfield.mirror.aliyuncs.com"]
          }
      }
  }
  我们所做的工作其实就是在第一行加上了"default-runtime": "nvidia",


  重启docker服务: sudo systemctl restart docker

  再次检查状态: systemctl status docker




/////////////////////////////////////////////////////////////
  



