
//////////////////////////////////////////////////////////////

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
	then link /apollo to /Cybertron2/apollo    // root@apollo5-cybertron2-ubuntu16-1:/Cybertron2# ln -s /apollo /Cybertron2/apollo


5. build apollo in ubuntu14 docker
	./apollo.sh  or  build_no_peception dbg (by default)    or   ./apollo.sh build_opt_gpu  // modify the WORKSPACE.in  https://gitee.com/audier0879/ad-rss-lib    //root@apollo5-apollo-ubuntu14-1:/apollo# ./apollo.sh build_no_peception dbg 

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

	root@apollo5-apollo-ubuntu14-1:/Cybertron2/Samples/5_DemoSceneSingleUeEditor/log#