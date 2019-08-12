 

7.  Cyber_monitor
 
		 1.start Dreamview
		 2.Dreamview->Module Controller -> Open Routing && Planning
		 3.Choose map
		 4.source /apollo/scripts/apolo_base.sh
		 5.Cyber_monitor



8. gdb debug

    1. ps aux | grep mainboard | grep planning


    root   432  5.4  0.7 4922856 233576 pts/20 Sl   06:12   0:20 mainboard -d /apollo/modules/planning/dag/planning.dag -p planning -s CYBER_DEFAULT


	2. sudo gdb -q bazel-bin/cyber/mainboard -p 432


	3. (gdb) b modules/planning/on_lane_planning.cc:374

	Breakpoint 1 at 0x7f3754207cb7: file modules/planning/on_lane_planning.cc, line 374.

	4. c : 运行到断点

    //因为Apollo 3.5以上版本通过动态创建的方式启动Planning模块，因此在使用GDB设置断点时，按下TAB键不会有提示，可以借助VSCode提供的Copy Relative Path功能撰写正确的源代码文件路径




 1. 在Docker内部编译Apollo项目
	使用GDB调试Apollo项目必须带有调试符号信息，因此编译Apollo项目时，不能使用opt选项，可根据实际需求使用如下两个编译命令中任意一个进行构建：

	# 构建方法1：使用8个线程（根据你的CPU核数确定）编译Apollo项目，不使用GPU也不优化
	bash apollo.sh build -j 8
	# 构建方法2：使用8个线程（根据你的CPU核数确定）编译Apollo项目，使用GPU但不优化
	bash apollo.sh build_gpu -j 8


2. Apollo 3.5以上版本使用Cyber RT进行任务调度与通信，调试功能模块的命令更新为（进入GDB后的操作方法相同）：
	# 启动方法1:
   gdb -q --args /apollo/bazel-bin/cyber/mainboard -d /apollo/modules/planning/dag/planning.dag


   # 启动方法2：在Dreamview中启动Planning模块，然后使用ps aux | grep planning命令查找
	# planning进程ID（PID），假设为35872，则使用attach模式附加到当前planning进程调试
	
	sudo gdb -q /apollo/bazel-bin/cyber/mainboard -p 35872



//在Docker内部使用GDB调试
gdb -q bazel-bin/modules/map/relative_map/navigation_lane_test

进入GDB调试界面后，使用l命令查看源代码，使用b 138在源代码第138行（可根据需要修改为自己所需的代码位置 ）设置断点，
使用r命令运行navigation_lane_test程序，进入断点暂停后，使用p navigation_lane_查看当前变量值（可根据需要修改为其他变量名），使用n单步调试一条语句，使用s单步调试进入函数内部，使用c继续执行后续程序。
如果哪个部分测试通不过，调试信息会立刻告诉你具体原因，可使用bt查看当前调用堆栈



3. 常见GDB调试命令

命令			作用				示例									解释

l		 查看源代码	       l 1,20	        				将GDB当前所在源程序的第1-20行列出来

b	     设置断点      	b planning.cc:164					在planning.cc文件第164行设置一个断点。注意：若GDB已调试进入源文件planning.cc中，则可以直接使用b 164命令以简化操作

b if	设置条件断点	    b planning.cc:662 if v > 10.0		若速度大于10.0m/s，则在planning.cc文件第662行设置一个断点

info b	显示当前断点	    info b	显示当前所有设置的断点

d		删除断点			d num								首先使用info b显示所有断点，然后删除第num个断点

clear	清除当前行的断点	clear 131							清除当前源文件中第131行的断点，不写行数表示当前行

set args	设置运行参数	set args --flagfile=/apollo/modules/planning/conf/planning.conf		设置一个运行参数flagfile，一般在进入GDB界面后，使用命令r运行进程前设置

show args	显示运行参数	show args							显示运行当前进程时从外面传入的参数

r		运行当前进程		r									在进入GDB调试界面并设置完断点后，使用该命令运行当前进程。注意：如果使用attach模式附加到已有进程PID调试，则不能使用r命令启动进程，而必须使用c命令继续执行当前进程

start	启动进程并停止在main函数入口处	start					启动进程并停止在main函数入口处

c	继续执行当前进程		c									进入某个断点后，使用该命令继续执行当前进程。 注意：如果使用attach模式附加到已有进程PID调试，必须使用c命令继续执行当前进程

n	单步执行				n									相当于VS中的F10，即每次执行一条语句，遇函数调用也当成一条普通语句直接返回执行结果

s	单步执行				s									相当于VS中的F11，即每次执行一条语句，遇函数调用则跳转进入调试

finish	停止调试当前函数	finish								停止调试当前函数跳转到该函数的下条语句，一般用于跳出当前函数调用，相当于Visual Studio中的Shift+F11，注意：不能简写为f，因为简写的f表示frame，即打印当前帧

until	跳出当前循环体		until								当你厌倦了在一个循环体内单步跟踪时，这个命令可以跳出当前循环体

until 行号	运行至某行，不只是跳出循环	until 341				运行至341行

info locals	显示当前的局部变量	info locals						显示当前调用堆栈中的所有局部变量

p	打印变量值	        p points.size()						打印points.size()的值

p	打印STL库容器中所有元素的值	p *(container._M_impl._M_start)@container.size()	打印STL库容器变量container的所有内容

p	打印STL库容器中前几个元素的值	p *(container._M_impl._M_start)@3	打印STL库容器变量container中前3个元素内容，注意@3不能超过容器的size

p	打印STL库容器中第几个元素的值	p *(container._M_impl._M_start+1)	打印STL库容器变量container中第二个元素内容，注意1不能超过容器的size-1

set var key = value	更改变量的值	set var init_val = 30	将变量init_val的值更改为30

p key=value	更改变量的值	p init_val = 30	将变量init_val的值更改为30， 与 set var init_val = 30作用相同

bt	显示调用堆栈					bt							显示调用堆栈，该语句在调试core dump文件时特别有用

Ctrl+c	停止当前的GDB指令	Ctrl+c	停止当前的GDB指令，退回GDB命令提示符。注意：如果使用attach模式附加到已有进程PID调试，可能无法退出当前执行的指令，可以通过kill PID命令停止被调试的进程，这时GDB会自动退出

Ctrl+d	退出GDB调试	Ctrl+d	退出GDB调试，与q作用相同。注意：如果使用attach模式附加到已有进程PID调试，可能无法退出GDB，可以通过kill PID命令停止被调试的进程，这时GDB会自动退出

q	退出GDB调试	q	退出GDB调试，与Ctrl+d作用相同。注意：如果使用attach模式附加到已有进程PID调试，可能无法退出GDB，可以通过kill PID命令停止被调试的进程，这时GDB会自动退出


4. GDB主要帮忙你完成下面四个方面的功能：

    1、启动你的程序，可以按照你的自定义的要求随心所欲的运行程序。
    2、可让被调试的程序在你所指定的调置的断点处停住。（断点可以是条件表达式）
    3、当程序被停住时，可以检查此时你的程序中所发生的事。
    4、动态的改变你程序的执行环境。


5.启动GDB的方法有以下几种：

    1、gdb <program> 
       program也就是你的执行文件，一般在当然目录下。

    2、gdb <program> core
       用gdb同时调试一个运行程序和core文件，core是程序非法执行后core dump后产生的文件。

    3、gdb <program> <PID>
       如果你的程序是一个服务程序，那么你可以指定这个服务程序运行时的进程ID。gdb会自动attach上去，并调试他。program应该在PATH环境变量中搜索得到




// VScode
1. Install extension: Remote - Containers
2. Open a remote window 
3. attach to a running container

4. 在进入GDB界面后，使用b命令设置相关断点，使用r命令启动待调试进程，待运行至断点处后，再根据具体需要合理使用n、s、c、p、bt等命令进行单步调试。
	注意：如果使用attach模式附加到已有进程PID调试，则不能使用r命令启动进程，而必须使用c命令继续执行当前进程。否则，GDB永远不会跳转至你所设置的断点处




5. Error loading config file XXX.dockerconfig.json permission denied
	在使用Docker容器技术的过程中，执行完某条命令可能会出现如下提示：

	Error loading config file XXX.dockerconfig.json - stat /home/XXX/.docker/config.json: permission denied

	这是因为docker的文件夹的权限问题导致的，处理办法如下，执行：

	sudo chown "$USER":"$USER" /home/"$USER"/.docker -R

	sudo chmod g+rwx "/home/$USER/.docker" -R



6. 

 ps -ef |grep planning


 root     26071     1  1 04:01 pts/22   00:00:00 /usr/bin/python /apollo/cyber/t
ools/cyber_launch/cyber_launch start /apollo/modules/planning/launch/planning.l
aunch
root     26095 26071  6 04:01 pts/22   00:00:02 mainboard -d /apollo/modules/pl
anning/dag/planning.dag -p planning -s CYBER_DEFAULT
root     26370 25732  0 04:02 pts/21   00:00:00 grep --color=auto planning


 gdb attach 26071



 