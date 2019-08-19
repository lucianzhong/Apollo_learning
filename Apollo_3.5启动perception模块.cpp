
// how to launch the perception module
	reference:
	https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_run_perception_module_on_your_local_computer.md
	https://blog.csdn.net/max_loo/article/details/81304949
	https://blog.csdn.net/davidhopper/article/details/85708221
	https://blog.csdn.net/weixin_41803874/article/details/96568869



// essential 
	cuda版本，nvidia驱动版本，卡的型号

	In this tutorial, it comes sucessfully with:
    cuda-8.0,NVIDIA-384,rtx1070




1. In Ubuntu need to intsall nvidia driver

	// 由于官方推荐安装NVIDIA-375版本的驱动  Apollo3.5   // 因为apollo镜像仅支持cuda8.0，不能使用太新版本的驱动程序
    // 如果遇到内核编译失败的问题，很大可能是驱动版本的问题，使用ppa的安装方法，然后确定ppa下载的是那个版本的驱动。确定了版本号，就可以手动安装

	///////////////////////////////////////
	// 方法 1（各方法等效，出错了，可以多替代尝试）
	
	// 首先看看自己Ubuntu内核支持到哪个版本的驱动
	sudo apt-cache search nvidia*

	// 如果安装的是官网下载的驱动,则重新运行run文件来卸载
    sh ./nvidia.run --uninstall
   // 卸载已存在驱动版本(可选)
   sudo apt-get remove --purge nvidia*
   // 卸载已存在驱动版本(可选)
   nvidia-uninstall


	Installing NVIDIA GPU Driver
	The Apollo runtime in the vehicle requires the NVIDIA GPU Driver. You must install the NVIDIA GPU driver with specific options.
	Download the installation files
	wget http://us.download.nvidia.com/XFree86/Linux-x86_64/375.39/NVIDIA-Linux-x86_64-375.39.run
	Start the driver installation
	sudo bash ./NVIDIA-Linux-x86_64-375.39.run --no-x-check -a -s --no-kernel-module

	// 查看显卡驱动是否安装成功
	nvidia-smi

	//In docker, nvidia-smi

	///////////////////////////////////////
	// 方法 2（各方法等效，出错了，可以多替代尝试）

    # 查看Docker当前驱动，没装过应该是没有
	sudo dpkg --list | grep nvidia-*
	# 如果已经安装过，并且与主机不符，需要先卸载老版本
	sudo apt-get remove nvidia-xxx

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


2. cuda-8.0 has been installed previously in docker. If the cuda-10.0 is not needed, ignore this step.

 	// cuda is linked to cuda8.0

	// install cuda, the .run file is download from nvidia website
	sudo ./cuda_10.1.168_418.67_linux.run 

		Please make sure that
	 -  PATH includes /usr/local/cuda-10.1/bin
	 -  LD_LIBRARY_PATH includes /usr/local/cuda-10.1/lib64, or, add /usr/local/cuda-10.1/lib64 to /etc/ld.so.conf and run ldconfig as root

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

	//cudnn
    //.  the cuDNN library installation path is /usr/lib/x86_64-linux-gnu/
    依照自己先前下載CUDA的版本，選擇配合的cuDNN版本，將三個deb檔案都下載下來，分別再執行

	$ sudo dpkg -i libcudnn7_7.1.4.18-1+cuda8.0_amd64.deb

	$ sudo dpkg -i libcudnn7-dev_7.1.4.18-1+cuda8.0_amd64.deb

	$ sudo dpkg -i libcudnn7-doc_7.1.4.18-1+cuda8.0_amd64.deb


	// docker copy 
    docker cp /home/zy/Downloads/libcudnn7-dev_7.1.4.18-1+cuda8.0_amd64.deb b7544a6273e0:/apollo



     # test cuda and cudnn
	cuda-install-samples-8.0.sh ~
	cd ~/NVIDIA_CUDA-8.0_Samples/0_Simple/cdpSimplePrint
	make  ./cdpSimplePrint

  
    // build apollo with gpu
    ./apollo.sh build_gpu  

    // without debug information	
    ./apollo.sh build_opt_gpu   

    // errors due to the lack of cudnn
         ERROR: (08-14 00:21:08.409) /apollo/modules/perception/base/BUILD:127:1: Linking of rule '//modules/perception/base:common_test' failed (Exit 1).
		/usr/bin/ld: cannot find -lnppi
		/usr/bin/ld: cannot find -lnppi



3. commit the docker image 

	docker commit container_id apolloauto/apollo:tag_name
	// container_id用docker ps -l查看, tag_name自己取名字。该命令用于将已基于源镜像更改的内容保存成新的镜像

	In docker:
	主机端：
	cd apollo#主机到阿波罗的根目录
	sudo bash docker/scripts/dev_start.sh -l -t tag_name#(对应之前commit的tag名称)
	sudo bash docker/scripts/dev_into.sh


4. // if it tells some error about the camera intrinsic
   // obs_sensor_intrinsic_path=/apollo/modules/perception/data/params

	mkdir /apollo/modules/perception/data/params/
	zy@in_dev_docker:/apollo$ sudo cp -r  /apollo/modules/perception/production/data/perception/camera/params/.  /apollo/modules/perception/data/params/


5. // launch the perception module

	source scripts/apollo_base.sh 

	cyber_launch start /apollo/modules/perception/production/launch/perception.launch

	// if below two commands give the "segmentation fault", one way to dag into the problem is the core dump file. Use the gdb tool to find which line generates error.
	// mostly,it is due to driver unmathch
	mainboard -d /apollo/modules/perception/production/dag/dag_streaming_perception.dag
    mainboard -d /apollo/modules/perception/production/dag/dag_streaming_perception_camera.dag -p perception -s CYBER_DEFAUL


    // gdb debug:
    gdb -q /apollo/bazel-bin/cyber/mainboard  /apollo/data/core/core_mainboard.16227	

	#0  0x00007fb3be0f8280 in apollo::perception::inference::RTNet::Init (this=0x20c2e00, shapes=...)
		    at modules/perception/inference/tensorrt/rt_net.cc:634
	634	  context_ = engine->createExecutionContext();


6.  // warnings:

	[perception]  [NVBLAS] NVBLAS_CONFIG_FILE environment variable is NOT set : relying on default config filename 'nvblas.conf'
	[perception]  [NVBLAS] Cannot open default config file 'nvblas.conf'
	[perception]  [NVBLAS] Config parsed
	[perception]  [NVBLAS] CPU Blas library need to be provided

	// how to slove warnings
	export NVBLAS_CONFIG_FILE=/usr/local/cuda
	export NVBLAS_CONFIG_FILE=/usr/local/cuda/nvblas.conf
	zy@in_dev_docker:/usr/local/cuda$ sudo touch nvblas.conf
	zy@in_dev_docker:/usr/local/cuda$ vim nvblas.conf   // put_in: NVBLAS_CPU_BLAS_LIB  /usr/lib/libopenblas.so 



7. 可执行一个demo检查是否成功编译GPU版本的APOLLO

	./bazel-bin/modules/perception/camera/test/camera_lib_obstacle_detector_yolo_region_output_test
	 //若出现 error while loading shared libraries: libcuda.so.1: cannot open shared object file: No such file or directory ,之类的错误提示，则说明docker 或者主机的NVIDIA驱动未正确安装


8. 直接运行所有测试
   
   ./apollo.sh test


9. how to run demo?
   
   source scripts/apollo_base.sh 
   cyber_launch start /apollo/modules/dreamview/launch/dreamview.launch 
   cyber_recorder play -f docs/demo_guide/demo_3.5.record --loop















	