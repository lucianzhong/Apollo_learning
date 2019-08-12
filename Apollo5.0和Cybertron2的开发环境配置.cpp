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

1. in Ubuntu need to intsall nvidia driver



	nvidia-smi


2. In docker:

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



 cyber_launch start /apollo/modules/perception/production/launch/perception.launch



 zy@in_dev_docker:/apollo$ sudo cp -r  /apollo/modules/perception/production/data/perception/camera/params  /apollo/modules/perception/production/data/perception/camera/params/
