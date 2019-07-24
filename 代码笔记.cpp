
// Cybertron/ Modules/ Unreal/ UnrealNodeBase/ CybertronUnrealNodeBase/ CMakeLists.txt


1. CybertronUnrealNodeBase is the a bridge between UE project and CybertronZero
 
 
2. add_library
   该指令的主要作用就是将指定的源文件生成链接文件，然后添加到工程中去
   而STATIC、SHARED和MODULE的作用是指定生成的库文件的类型。
   STATIC库是目标文件的归档文件，在链接其它目标的时候使用。
   SHARED库会被动态链接（动态链接库），在运行时会被加载。
   MODULE库是一种不会被链接到其它目标中的插件，但是可能会在运行时使用dlopen-系列的函数。默认状态下，库文件将会在于源文件目录树的构建目录树的位置被创建，该命令也会在这里被调用。
   
   add_library(CybertronUnrealNodeBase SHARED "")
   
3. IF (WIN32)
	add_definitions(-D_WIN32_WINNT=0x0501)
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi")
	set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DEBUG /OPT:REF /OPT:ICF")
    ENDIF()

4.    # Add the NOMINMAX macro to avoid Windows' min and max macros.
    ADD_DEFINITIONS(-DNOMINMAX)
	
	add_definitions：添加编译参数

	add_definitions(-DDEBUG)将在gcc命令行添加DEBUG宏定义；
	
5. foreach  对一个list中的每一个变量执行一组命令。

  foreach(loop_var arg1 arg2 ...)
    COMMAND1(ARGS ...)
    COMMAND2(ARGS ...)
    ...
  endforeach(loop_var)
  
  
6. file(GLOB_RECURSE variable [RELATIVE path] 
       [FOLLOW_SYMLINKS] [globbing expressions]...)
	   
GLOB_RECURSE选项将会生成一个类似于通常的GLOB选项的list，只是它会寻访所有那些匹配目录的子路径并同时匹配查询表达式的文件。
作为符号链接的子路径只有在给定FOLLOW_SYMLINKS选项或者cmake策略CMP0009被设置为NEW时，才会被寻访到。参见cmake --help-policy CMP0009 查询跟多有用的信息。

7. 在visual studio中我们可以添加筛选器给源文件进行分组。cmake提供了source_group()来支持这一特性。
    source_group需要写在add_library或者add_executable之后 
	
8. target_sources(<target><INTERFACE|PUBLIC|PRIVATE> [items1...][<INTERFACE|PUBLIC|PRIVATE> [items2...] ...])
往一个目标里面添加源文件，这个目标名字target是在add_executable() 或者add_library() 中定义的name



9. TARGET_LINK_LIBRARIES （设置要链接的库文件的名称）
	语法：TARGET_LINK_LIBRARIES(targetlibrary1 <debug | optimized> library2 ..)

	比如（以下写法（包括备注中的）都可以）： 
	TARGET_LINK_LIBRARIES(myProject hello)，连接libhello.so库
	TARGET_LINK_LIBRARIES(myProject libhello.a)
	TARGET_LINK_LIBRARIES(myProject libhello.so)


10.第二种形式是为某个目标如库或可执行程序添加一个客制命令。这对于要在构建一个目标之前或之后执行一些操作非常有用。该命令本身会成为目标的一部分，仅在目标本身被构建时才会执行。如果该目标已经构建，命令将不会执行。

 
11. 
 add_custom_command(TARGET target
                     PRE_BUILD | PRE_LINK| POST_BUILD
                     COMMAND command1[ARGS] [args1...]
                     [COMMAND command2[ARGS] [args2...] ...]
                     [WORKING_DIRECTORYdir]
                     [COMMENT comment][VERBATIM])

命令执行的时机由如下参数决定： 

PRE_BUILD - 命令将会在其他依赖项执行前执行
  PRE_LINK - 命令将会在其他依赖项执行完后执行
  POST_BUILD - 命令将会在目标构建完后执行。




12.  
D:\Cybertron\Modules\Unreal\UnrealNodeBase

DataStruct.h 

  static constexpr const char* PedestrianLandmark[] = {
        "LeftEar", "RightEar", "LeftShoulder", "RightShoulder", "LeftForeArm", "RightForeArm",
        "LeftHand", "RightHand", "LeftUpLeg", "RightUpLeg", "LeftLeg", "RightLeg",
        "LeftFoot", "RightFoot"
        };


13. 
表示引用，引用和指针类似，但引用可以理解为同一个对象的不同命名，而且引用必须初始化，不能重复定义。还有引用不会分配空间

UnrealBridge::MainVehicle::MainVehicleData &from


14. 
#pragma comment ( lib,"wpcap.lib" )

表示链接wpcap.lib这个库。
和在工程设置里写上链入wpcap.lib的效果一样，不过这种方法写的 程序别人在使用你的代码的时候就
不用再设置工程settings了。告诉连接器连接的时候要找ws2_32.lib，这样你就不用在linker的lib设置
里指定这个lib了。


15. 
warning C4996: strcpy was declared deprecated 
在使用VC 2005 的开发者会遇到这样的问题，在使用std命名空间库函数的时候，往往会出现类似于下面的警告：    warning C4996: strcpy was declared deprecated
出现这样的警告，是因为VC2005中认为CRT中的一组函数如果使用不当，可能会产生诸如内存泄露、缓冲区溢出、非法访问等安全问题。这些函数如：strcpy、strcat等。
对于这些问题，VC2005建议使用这些函数的更高级的安全版本，即在这些函数名后面加了一个_s的函数。这些安全版本函数使用起来更有效，也便于识别，如：strcpy_s,calloc_s等。
当然，如果执意使用老版本、非安全版本函数，可以使用_CRT_SECURE_NO_DEPRECATE标记来忽略这些警告问题。办法是在编译选项 C/C++ | Preprocessor | Preprocessor Definitions中，增加_CRT_SECURE_NO_DEPRECATE标记即可。或在程序开头添加          
#pragma  warning(disable:4996)   //全部关掉          #pragma  warning(once:4996)      //仅显示一个


16.
C++11的新特性——可调用对象模板类。一句话说明问题，std::function<Layer*()>代表一个可调用对象，接收0个参数，返回Layer*
std::function<void(int, int, int, const uint8_t*, const UnrealBridge::SensorHeader*, const UnrealBridge::GroundTruthData*)> imageEncodedCallBack) :
	mbInited(false),
	mWidth(width),
	mHeight(height),
	mStreamUrl(streamUrl),
	mImageEncodedCallBack(imageEncodedCallBack),
	mImageSize(width * height * 4),
	mImageQueueSize(0),
	mStopFlag(false)


17.std::thread thread(&FFmpeger::Run, this);

18. memmove() 与 memcpy() 类似都是用来复制 src 所指的内存内容前 num 个字节到 dest 所指的地址上。
    不同的是，memmove() 更为灵活，当src 和 dest 所指的内存区域重叠时，memmove() 仍然可以正确的处理，不过执行效率上会比使用 memcpy() 略慢些

19. C++ 11中使用nullptr来表示空指针



20.
	struct SensorTrafficLight : public SensorActor
	{
		SensorTrafficLight(const TrafficLight& trafficLight);
		void ToTrafficLight(TrafficLight& trafficLight) const;

		int index;
		std::string colorString;
	};
	typedef std::vector<SensorTrafficLight> SensorTrafficLights;


	SensorTrafficLights						TrafficLights;



21. c++ thread

22.
	for (const auto& trafficTwoWheelVehicle : trafficTwoWheelVehicles) {
		pCurrentFrame->TrafficTwoWheelVehicles.emplace_back(UnrealBridge::SensorTrafficTwoWheelVehicle(trafficTwoWheelVehicle));
	}
	
	
23.























































































