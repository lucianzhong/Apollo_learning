ls
1. rancher									// ubuntu & window need different environment for each other in rancher
	cybertron2.0 environment
		apollo: 5.0
		windows: false
		port: 8889
		name: apollo5

2. update ubuntu16 docker on rancher
	volume
		apollo5_cache-data_20e43:/root/.cache   // Add Volume
		use name: root

3. Need to download Cybertron2 manually   // git config --global credential.helper store   // repeatedly inputs of password


4. rm submodule /Cybertron2/apollo
	then link /apollo to /Cybertron2/apollo    //root@apollo5-cybertron2-ubuntu16-1:/Cybertron2# ln -s /apollo /Cybertron2/apollo


5. build apollo in ubuntu14 docker
	./apollo.sh  build_no_peception dbg (by default)    or   ./apollo.sh build_opt_gpu  // modify the WORKSPACE.in  https://gitee.com/audier0879/ad-rss-lib    //root@apollo5-apollo-ubuntu14-1:/apollo# ./apollo.sh build_no_peception dbg 


	root@apollo5-apollo-ubuntu14-1:/apollo/bazel-bin/CyberBridge
	CybertronBridgeApollo


	
6. build cybertron and apollo cyber bridge

	root@apollo5-cybertron2-ubuntu16-1:/Cybertron2/Modules/Foundation/Build#
		./rebuild_debug.sh
	root@apollo5-cybertron2-ubuntu16-1:/Cybertron2/Modules/BridgeApollo/CybertronBridgeApolloCyber#   // need the built  ../../../../apollo/bazel-bin/CyberBridge/libApolloCyber.so
		./rebuild_debug.sh



7. data log location
	root@apollo5-apollo-ubuntu14-1:/apollo/data/log


8.
root@apollo5-apollo-ubuntu14-1:/Cybertron2/Samples/Common# 
vim /Cybertron2/Samples/Common/Config.sh
	export CoordinatorIp=10.2.35.104

 source ./scripts/apollo_base.sh


		run apollo and dreamview
		/apollo
			kill.sh
			run.sh

standalone dreamview
	cyber_launch start modules/dreamview/launch/dreamview.launch



9. apollo5-apollo-ubuntu14
	apt-get install libc++-dev libc++abi-dev


10. 

root@apollo5-cybertron2-ubuntu16-1:/Cybertron2/Modules/Foundation/Build/build_debug/bin# ln -s /Cybertron2/apollo/bazel-bin/CyberBridge/CybertronBridgeApollo ./CybertronBridgeApollo


root@apollo5-cybertron2-ubuntu16-1:/Cybertron2/Modules/Foundation/Build/build_debug/bin# ls -al
total 297244
drwxr-xr-x  3 root root     4096 Aug  7 08:59 .
drwxr-xr-x 29 root root     4096 Aug  7 08:39 ..
lrwxrwxrwx  1 root root       62 Aug  7 08:59 CybertronBridgeApollo -> /Cybertron2/apollo/bazel-bin/CyberBridge/CybertronBridgeApollo




11. 
	11.1  \apollo bridge
		  root@apollo5-apollo-ubuntu14-1:/Cybertron2/Samples/5_DemoSceneSingleUeEditor ./StartApolloBridgeDaemon.sh 

 	11.2 run apollo and dreamview /apollo 	run.sh


12.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ls
code-insiders
	remote-containers
	remote-ssh
	c/c++
	c++ intellisense

attach container
	the attach to an apollo module with gdb
		"program": "${workspaceRoot}/bazel-bin/cyber/mainboard",
		"sourceFileMap": {
			"/proc/self/cwd": "/apollo"
		}, 

map setting from cybertron
root@apollo5-apollo-ubuntu14-1:/apollo# git diff modules/common/data/global_flagfile.txt
	--map_dir=/apollo/modules/map/data/jiading

/apollo/scripts/kill.sh
	triggered to reload map sent from cybertron
	or relaunch /apollo/run.sh

cyber_monitor
root@apollo5-apollo-ubuntu14-1:/apollo# ps -ef|grep routing



debug with apollo only
	enable sim control
	enable planning and routing modules
	route edit, then send routing request




13.

////////////////////////////////////////////////////////////////

	root@apollo5-cybertron2-ubuntu16-1:/root/.cache# ls -ltra

ll;



/////////////////////////

root@apollo5-apollo-ubuntu14-1:/apollo# cyber_launch start /apollo/modules/perception/production/launch/perception.launch



    ./scripts/monitor.sh start
    ./scripts/dreamview.sh start





cyber_launch start /apollo/modules/perception/production/launch/perception_all.launch  ????????




source /home/tmp/ros/setup.bash
SamplePath=$(pwd)
source ../Common/Config.sh
Port=4500
cmd="$SamplePath/$FoundationBinaryPath/CybertronDaemon ApolloBridge -f $SamplePath/DaemonApolloBridge.json -h $CoordinatorIp -p $Port"
echo $cmd
$cmd
#xterm -e "$cmd" &


// /Cybertron2/Samples/5_DemoSceneSingleUeEditor/../../Modules/Foundation/Build/build_debug/bin/CybertronDaemon ApolloBridge -f /Cybertron2/Samples/5_DemoSceneSingleUeEditor/DaemonApolloBridge.json -h 10.2.35.156 -p 4500
   




14. 
  ps -ef | grep CybertronBridgeApollo


  gdb

  (gdb) attach 27501




gdb调试正在运行的进程：

GDB可以对正在执行的程序进行调度，它允许开发人员中断程序 并查看其状态，之后还能让这个程序正常地继续执行

info proc显示当前程序可执行文件相关信息（name，pwd）



15. root@apollo5-apollo-ubuntu14-1:/apollo/bazel-bin/CyberBridge# gdb CybertronBridgeApollo 
    
    root@apollo5-apollo-ubuntu14-1:/apollo/bazel-out/local-opt/bin/CyberBridge# gdb CybertronBridgeApollo   

    run




     b CyberWriterImu.cpp:25






16. gdb -q --args /apollo/bazel-bin/cyber/mainboard -d /apollo/modules/planning/dag/planning.dag

	(gdb) run




17  (gdb) b CyberWriterImu.cpp: 30      
	Breakpoint 2 at 0x7f9338bb8821: file CyberBridge/CyberWriterImu.cpp, line 30.
	(gdb) c
	Continuing.
	[Switching to Thread 0x7f92a77fe700 (LWP 2633)]

	Breakpoint 2, CyberWriterImu::publish (this=0x29d3e20, angVelX=0, angVelY=0, angVelZ=0, linAccX=0, linAccY=0, linAccZ=0) at CyberBridge/CyberWriterImu.cpp:30
	30		imu->mutable_imu()->mutable_linear_acceleration()->set_y(linAccY);




/////////////////////////////////
// how to launch the perception module
	reference:
	https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_run_perception_module_on_your_local_computer.md

1. in Ubuntu need to intsall nvidia driver
					// 由于官方推荐安装NVIDIA-375版本的驱动  Apollo3.5   // 因为apollo镜像仅支持cuda8.0，不能使用太新版本的驱动程序

	// 首先看看自己Ubuntu内核支持到哪个版本的驱动
	sudo apt-cache search nvidia*

	// 如果安装的是官网下载的驱动,则重新运行run文件来卸载
    sh ./nvidia.run --uninstall

   // 卸载已存在驱动版本(可选)
sudo apt-get remove --purge nvidia*


					Installing NVIDIA GPU Driver
			The Apollo runtime in the vehicle requires the NVIDIA GPU Driver. You must install the NVIDIA GPU driver with specific options.

			Download the installation files
			wget http://us.download.nvidia.com/XFree86/Linux-x86_64/375.39/NVIDIA-Linux-x86_64-375.39.run
			Start the driver installation
			sudo bash ./NVIDIA-Linux-x86_64-375.39.run --no-x-check -a -s --no-kernel-module



	nvidia-smi


2. 
	# 查看Docker当前驱动，没装过应该是没有
	sudo dpkg --list | grep nvidia-*
	# 如果已经安装过，并且与主机不符，需要先卸载老版本
	sudo apt-get remove nvidia-xxx

3.  In docker:



		主机端：
		cd apollo#主机到阿波罗的根目录
		sudo bash docker/scripts/dev_start.sh -l -t tag_name#(对应之前commit的tag名称)
		sudo bash docker/scripts/dev_into.sh



	# 启动并进入Docker
	bash docker/scripts/dev_start.sh -C
	bash docker/scripts/dev_into.sh
	# 更新安装源
	sudo add-apt-repository ppa:graphics-drivers/ppa
	sudo apt update
	# 查看并安装NVIDIA显卡驱动
	# 此处的版本号要与Docker外部一致！
	apt search nvidia-430
	sudo apt install nvidia-430
	# 查看显卡驱动是否安装成功
	nvidia-smi


 // cuda is linked to cuda8.0

		sudo ./cuda_10.1.168_418.67_linux.run 

	Please make sure that
 -   PATH includes /usr/local/cuda-10.1/bin
 -   LD_LIBRARY_PATH includes /usr/local/cuda-10.1/lib64, or, add /usr/local/cuda-10.1/lib64 to /etc/ld.so.conf and run ldconfig as root


		 //配置到环境变量（不同环境下，配置不同环境变量，可以使用多个cuda版本）
		sudo gedit ~/.bashrc
		打开文件后在文件末尾添加路径，也就是安装目录，命令如下：

		export PATH=/usr/local/cuda-10.0/bin:$PATH
		export LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64:$LD_LIBRARY_PATH　

      export CUDA_HOME=$CUDA_HOME:/usr/local/cuda-10.0
		
		source ~/.bashrc


sudo rm -rf cuda
sudo ln -s /usr/local/cuda-8.0/ /usr/local/cuda
		


		 ls /usr/local/cuda
         
         nvcc -V

///////////////////////////////////
//cudnn


         //.  the cuDNN library installation path is /usr/lib/x86_64-linux-gnu/
         依照自己先前下載CUDA的版本，選擇配合的cuDNN版本，將三個deb檔案都下載下來，分別再執行

$ sudo dpkg -i libcudnn7_7.1.4.18-1+cuda8.0_amd64.deb

$ sudo dpkg -i libcudnn7-dev_7.1.4.18-1+cuda8.0_amd64.deb

$ sudo dpkg -i libcudnn7-doc_7.1.4.18-1+cuda8.0_amd64.deb




         docker cp /home/zy/Downloads/libcudnn7-dev_7.1.4.18-1+cuda8.0_amd64.deb b7544a6273e0:/apollo
docker cp /home/zy/Downloads/libcudnn7_7.1.4.18-1+cuda8.0_amd64.deb  b7544a6273e0:/apollo


         	# test cuda and cudnn
cuda-install-samples-8.0.sh ~
		cd ~/NVIDIA_CUDA-8.0_Samples/0_Simple/cdpSimplePrint
		make
		./cdpSimplePrint

  4.  
  // build apollo with gpu
  ./apollo.sh build_gpu    


  		// without debug information	
        ./apollo.sh build_opt_gpu   

         ERROR: (08-14 00:21:08.409) /apollo/modules/perception/base/BUILD:127:1: Linking of rule '//modules/perception/base:common_test' failed (Exit 1).
		/usr/bin/ld: cannot find -lnppi
		/usr/bin/ld: cannot find -lnppi



	5.
			obs_sensor_intrinsic_path=/apollo/modules/perception/data/params
			mkdir /apollo/modules/perception/data/params/

		 zy@in_dev_docker:/apollo$ sudo cp -r  /apollo/modules/perception/production/data/perception/camera/params/.  /apollo/modules/perception/data/params/



6.
				[perception]  [NVBLAS] NVBLAS_CONFIG_FILE environment variable is NOT set : relying on default config filename 'nvblas.conf'
				[perception]  [NVBLAS] Cannot open default config file 'nvblas.conf'
				[perception]  [NVBLAS] Config parsed
				[perception]  [NVBLAS] CPU Blas library need to be provided

				export NVBLAS_CONFIG_FILE=/usr/local/cuda

				export NVBLAS_CONFIG_FILE=/usr/local/cuda/nvblas.conf

				zy@in_dev_docker:/usr/local/cuda$ sudo touch nvblas.conf


				zy@in_dev_docker:/usr/local/cuda$ vim nvblas.conf   // put_in: NVBLAS_CPU_BLAS_LIB  /usr/lib/libopenblas.so 



		7.  docker commit container_id apolloauto/apollo:tag_name
			#container_id用docker ps -l查看, tag_name自己取名字。该命令用于将已基于源镜像更改的内容保存成新的镜像




		8.

		source scripts/apollo_base.sh 

		 cyber_launch start /apollo/modules/perception/production/launch/perception.launch



		mainboard -d /apollo/modules/perception/production/dag/dag_streaming_perception.dag


		 mainboard -d /apollo/modules/perception/production/dag/dag_streaming_perception_camera.dag -p perception -s CYBER_DEFAUL





	9.   


		gdb -q /apollo/bazel-bin/cyber/mainboard  /apollo/data/core/core_mainboard.16227



		

		core_mainboard.25475


		#0  0x00007fb3be0f8280 in apollo::perception::inference::RTNet::Init (this=0x20c2e00, shapes=...)
		    at modules/perception/inference/tensorrt/rt_net.cc:634
		634	  context_ = engine->createExecutionContext();
nv