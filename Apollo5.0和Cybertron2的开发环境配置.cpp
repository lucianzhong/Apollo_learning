
//////////////////////////////////////////////////////////////

1. rancher									// ubuntu & window need different environment for each other in rancher
	cybertron2.0 environment
		apollo: 5.0
		windows: false
		port:24800 // default
		port: 8889
		name: apollo5

2. update ubuntu16 docker on rancher
	volume
		apollo5_cache-data_20e43:/root/.cache   // Add Volume   // In apollo5-apollo-ubuntu14-1,  volume: apollo5_cache-data_20e43 	/root/.cache, add to ubuntu16, upgrade add volume: apollo5_cache-data_20e43 	/root/.cache  //  Finsh update,the docekr exec need to start again

		use name: root

3. Need to download Cybertron2 manually   // ubuntu16 // git config --global credential.helper store   // repeatedly inputs of password
										// Download Cybertron, copy file in Cyberstron to Cybertron2 // sudo cp -rf Cybertron/* Cybertron2/

4. rm submodule /Cybertron2/apollo
	then link /apollo to /Cybertron2/apollo    // root@apollo5-cybertron2-ubuntu16-1:/Cybertron2# ln -s /apollo /Cybertron2/apollo


5. build apollo in ubuntu14 docker
	./apollo.sh  or  build_no_peception dbg (by default)    or   ./apollo.sh build_opt_gpu  // modify the WORKSPACE.in  https://gitee.com/audier0879/ad-rss-lib    //root@apollo5-apollo-ubuntu14-1:/apollo# ./apollo.sh build_no_peception dbg 




	
6. build cybertron and apollo cyber bridge

	root@apollo5-cybertron2-ubuntu16-1:/Cybertron2/Modules/Foundation/Build#
		sudo ./rebuild_debug.sh
	root@apollo5-cybertron2-ubuntu16-1:/Cybertron2/Modules/BridgeApollo/CybertronBridgeApolloCyber#   // need the built  ../../../../apollo/bazel-bin/CyberBridge/libApolloCyber.so
		sudo ./rebuild_debug.sh





7. data log location
	root@apollo5-apollo-ubuntu14-1:/apollo/data/log


8.
	root@apollo5-apollo-ubuntu14-1:/Cybertron2/Samples/Common# 
																	vim /Cybertron2/Samples/Common/Config.sh
																		export CoordinatorIp=10.2.35.104




9. apollo5-apollo-ubuntu14:
	apt-get install libc++-dev libc++abi-dev


10. 
root@apollo5-cybertron2-ubuntu16-1:/Cybertron2/Modules/Foundation/Build/build_debug/bin# ln -s /Cybertron2/apollo/bazel-bin/CyberBridge/CybertronBridgeApollo ./CybertronBridgeApollo

root@apollo5-cybertron2-ubuntu16-1:/Cybertron2/Modules/Foundation/Build/build_debug/bin# ls -al
total 297244
drwxr-xr-x  3 root root     4096 Aug  7 08:59 .
drwxr-xr-x 29 root root     4096 Aug  7 08:39 ..
lrwxrwxrwx  1 root root       62 Aug  7 08:59 CybertronBridgeApollo -> /Cybertron2/apollo/bazel-bin/CyberBridge/CybertronBridgeApollo




11. 
	run apollo bridge

	root@apollo5-apollo-ubuntu14-1:/Cybertron2/Samples/5_DemoSceneSingleUeEditor ./StartApolloBridgeDaemon.sh 



12.  run apollo and dreamview /apollo 	run.sh

       
	 source ./scripts/apollo_base.sh


		run apollo and dreamview
		/apollo
			kill.sh
			run.sh

	standalone dreamview launch:
		cyber_launch start modules/dreamview/launch/dreamview.launch


////////////////////////////////////////////////////////////////////////////////////////////////////


// use preStart.py to generate sim_map and routing_map

python preStart.py  --url=  --md5=  --loop=0  --scene_name=aachen



// How to debug the Bridage data:

1. 
	run apollo bridge

	root@apollo5-apollo-ubuntu14-1:/Cybertron2/Samples/5_DemoSceneSingleUeEditor ./StartApolloBridgeDaemon.sh    // make sure the daemon has sucessfully subscribe hot area
																												 // root       576   557  3 13:08 pts/27   00:00:02 /Cybertron2/Modules/Foundation/Build/build_debug/bin/CybertronBridgeApollo --port 24750
																												 // add_executable(CybertronBridgeApollo "")   // CMakeLists.txt  // Step 6

	ps -ef | grep CybertronBridgeApollo  			

  	gdb

 	(gdb) attach 27501

 	(gdb) info proc


 	


2. the binary files:
	
	root@apollo5-apollo-ubuntu14-1:/Cybertron2/Modules/Foundation/Build/build_debug/bin
	
	//root       576   557  3 13:08 pts/27   00:00:02 /Cybertron2/Modules/Foundation/Build/build_debug/bin/CybertronBridgeApollo --port 24750
	

3. 
	root@apollo5-apollo-ubuntu14-1:/Cybertron2/Samples/5_DemoSceneSingleUeEditor# ./StartApolloBridgeDaemon.sh 

	/Cybertron2/Samples/5_DemoSceneSingleUeEditor/../../Modules/Foundation/Build/build_debug/bin/CybertronDaemon ApolloBridge -f /Cybertron2/Samples/5_DemoSceneSingleUeEditor/DaemonApolloBridge.json -h 10.2.35.156 -p 4500



4. // log files

	// apollo/cyber/logger/log_file_object.cc
	apollo_publish_obstacles.INFO  
	bridge.INFO

 	apollo::cyber::Init("apollo_publish_obstacles");
 

// the Cybertron2 log files are in the log folder
   apollo/CyberBridge/CyberWriterObstacles.cpp
	root@apollo5-apollo-ubuntu14-1:/Cybertron2/Samples/5_DemoSceneSingleUeEditor/log#
	CybertronDaemon_ApolloBridge
	CybertronBridgeApolloCyber




5. 
// cyber_recorder

// 录制
root@apollo5-apollo-ubuntu14-1:/apollo# cyber_recorder record -a
// 查看录制信息
root@apollo5-apollo-ubuntu14-1:/apollo# cyber_recorder info 20190819115806.record.00002  

// 回放和暂停
root@apollo5-apollo-ubuntu14-1:/apollo# cyber_recorder play -f 20190819115806.record.00002


// 查看channel
root@apollo5-apollo-ubuntu14-1:/apollo# cyber_channel list

// 查看channel的内容
cyber_channel echo /apollo/planning
cyber_channel echo /apollo/perception/obstacles


// 回放和暂停
root@apollo5-apollo-ubuntu14-1:/apollo# cyber_recorder play -f 20190819115806.record.00002




6.   检查现有文件监控数目
cat /proc/sys/fs/inotify/max_user_watches

修改文件监控数目
综合考虑实际需监控文件的数目和内存消耗情况，我将新的文件监控数目设置为：81920，即原来监控数目的10倍。我使用vi对配置文件进行编辑：

sudo vi /etc/sysctl.conf


在该配置文件的最后一行加上下述语句：

fs.inotify.max_user_watches=54288
1
3. 让配置文件中的新文件监控数目生效
sudo sysctl -p


重新打开VSCode，只要当前文件夹内文件数目不超过81920个，就不会再出现警告信息了。