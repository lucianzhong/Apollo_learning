
1. 在终端窗口输入meld或搜索框里面搜索meld，即可启动该软件


2.  source /cyber/setup.bash 

 	cyber_channel list //列出所有活动的topic



3. 多出和改动的文件:
	
	1. readthedocs.yml
	2. run.sh
	3. kill.sh
	4. generatemap.sh
	5. buildrelease.sh
	6. apollo.sh
	7.
	8.
	9.
	10.



4. run.sh

 	export LD_LIBRARY_PATH=/usr/local/lib64:$LD_LIBRARY_PATH							//export [-fnp][变量名称]=[变量设置值]
	source /apollo/scripts/apollo_base.sh
	cyber_launch start /apollo/modules/localization/launch/localization.launch  &
	sleep 1																				//睡眠1秒
	cyber_launch start /apollo/modules/prediction/launch/prediction.launch  &
	sleep 1
	cyber_launch start /apollo/modules/routing/launch/routing.launch  &
	sleep 1
	cyber_launch start /apollo/modules/planning/launch/planning.launch  &
	sleep 1
	cyber_launch start /apollo/modules/control/launch/control.launch  &
	sleep 1
	cyber_launch start /apollo/modules/dreamview/launch/dreamview.launch  &



5. kill.sh


	ps -ef | grep /apollo/cyber | grep -v -E 'grep' | awk -F " " '{print $2}' | xargs kill -7
	ps -ef | grep mainboard 	| grep -v -E 'grep' | awk -F " " '{print $2}' | xargs kill -7
	ps -ef | grep dreamview 	| grep -v -E 'grep' | awk -F " " '{print $2}' | xargs kill -7


	// ps -e 显示所有进程, -f 全格式
 	// grep -v或 --revert-match : 显示不包含匹配文本的所有行, -E 或 --extended-regexp : 将样式为延伸的普通表示法来使用
 	// awk就是把文件逐行的读入，以空格为默认分隔符将每行切片，切开的部分再进行各种分析处理,


 6. generatemap.sh


	 MAPROOT="/apollo/modules/map/data"

	echo -e "\n~~~Genrate apollo routing map and sim map~~~\n"

	if [ "x$1" == "x" ]; then 
	  read -p "Do you wish to re-generate all maps? (y/n)" yn
	  if [ "$yn" != "${yn#[Yy]}" ] ;then
	    MAPDIRS=`ls -d $MAPROOT/*/` 
	    for MAPDIR in $MAPDIRS
	      do
	         echo "generating map for $MAPDIR"
	         ./scripts/generate_routing_topo_graph.sh --map_dir $MAPDIR
	         ./bazel-bin/modules/map/tools/sim_map_generator --map_dir=$MAPDIR --output_dir=$MAPDIR
	    done
	  fi
	else
	  MAPDIR="${MAPROOT}/$1"
	  echo "generating map for $MAPDIR"
	  ./scripts/generate_routing_topo_graph.sh --map_dir $MAPDIR 
	  ./bazel-bin/modules/map/tools/sim_map_generator --map_dir=$MAPDIR --output_dir=$MAPDIR
	fi


7.  buildrelease.sh

	echo "build release"
	cd /apollo
	./apollo.sh release
	echo "creating missing link"
	cd /root/.cache/apollo_release/apollo/bazel-bin/_solib_k8/_U@fastrtps_S_S_Cfastrtps___Uexternal_Sfastrtps_Slib
	ln -s /usr/local/fast-rtps/lib/libfastcdr.so libfastcdr.so -f
	ln -s /usr/local/fast-rtps/lib/libfastcdr.so.1 libfastcdr.so.1 -f
	ln -s /usr/local/fast-rtps/lib/libfastrtps.so libfastrtps.so -f
	ln -s /usr/local/fast-rtps/lib/libfastrtps.so.1 libfastrtps.so.1 -f
	echo "copy files"
	cp -rf /apollo/bazel-bin/CyberBridge /root/.cache/apollo_release/apollo/bazel-bin
	cp /Cybertron2/Modules/BridgeApollo/Config/* /root/.cache/apollo_release/apollo/bazel-bin/CyberBridge
	cp -rf /apollo/modules/map/data /root/.cache/apollo_release/apollo/modules/map
	cd /root/.cache/apollo_release/apollo
	tar -cvzf /exchange/a5.tar.gz .


8. apollo.sh

 function generate_build_targets() {
  COMMON_TARGETS="//cyber/... union //modules/common/kv_db/... union //modules/dreamview/... union //CyberBridge/..."


 BUILD_TARGETS=`bazel query //modules/... except //modules/perception/... union //cyber/... union //CyberBridge/...`


 BUILD_TARGETS=`bazel query //modules/... union //cyber/... union //CyberBridge/...`






9.
















10.


















11.



















12.

