参考：
https://blog.csdn.net/davidhopper/article/details/85248799



// DreamView模块启动过程
	
1.先从启动脚本文件scripts/bootstrap.sh开始剖析。服务启动命令bash scripts/bootstrap.sh start实际上执行了scripts/bootstrap.sh脚本中的start函数，
  start函数内部分别调用脚本文件scripts/monitor.sh与scripts/dreamview.sh内部的start函数启动monitor与dreamview模块
	function start() {
    ./scripts/monitor.sh start
    ./scripts/dreamview.sh start
    if [ $? -eq 0 ]; then
        http_status="$(curl -o -I -L -s -w '%{http_code}' ${DREAMVIEW_URL})"
        if [ $http_status -eq 200 ]; then
            echo "Dreamview is running at" $DREAMVIEW_URL
        else
            echo "Failed to start Dreamview. Please check /apollo/data/log or /apollo/data/core for more information"
        fi
    fi
	}

2. scripts/dreamview.sh：apollo_base.sh脚本文件，并且有一条调用语句：run dreamview "$@"（展开以后就是run dreamview start），run函数存在于apollo_base.sh脚本文件

	DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
	cd "${DIR}/.."
	source "${DIR}/apollo_base.sh"
	# run function from apollo_base.sh
	# run command_name module_name
	run dreamview "$@"
	
3. apollo_base.sh:

	function run() {
	  local module=$1
	  shift
	  run_customized_path $module $module "$@"
	}
	
	module的值为dreamview，$@的值为start, run_customized_path dreamview dreamview start 
	
4. apollo_base.sh:	
	function run_customized_path() {
	  local module_path=$1
	  local module=$2
	  local cmd=$3
	  shift 3
	  case $cmd in
		start)
		  start_customized_path $module_path $module "$@"
		  ;;
	   # ...
	}
  
  
	start_customized_path $module_path $module "$@",实际调用的是start_customized_path dreamview dreamview
	
	
	
5. apollo_base.sh:	
	function start_customized_path() {
	  MODULE_PATH=$1
	  MODULE=$2
	  shift 2

	  is_stopped_customized_path "${MODULE_PATH}" "${MODULE}"
	  if [ $? -eq 1 ]; then
		eval "nohup cyber_launch start /apollo/modules/${MODULE_PATH}/launch/${MODULE}.launch &"
		sleep 0.5
		is_stopped_customized_path "${MODULE_PATH}" "${MODULE}"
		if [ $? -eq 0 ]; then
		  echo "Launched module ${MODULE}."
		  return 0
		else
		  echo "Could not launch module ${MODULE}. Is it already built?"
		  return 1
		fi
	  else
		echo "Module ${MODULE} is already running - skipping."
		return 2
	  fi
	}
	
	
	在start_customized_path函数内部，首先调用is_stopped_customized_path函数来判断（在内部实际通过指令$(pgrep -c -f "modules/dreamview/launch/dreamview.launch")来判断）dreamview模块是否已启动。
	若该模块未启动，则使用指令nohup cyber_launch start /apollo/modules/dreamview/launch/dreamview.launch &以非挂断方式启动后台进程模块dreamview

	cyber_launch是Cyber平台提供的一个python工具程序，其完整路径为：${APOLLO_HOME}/cyber/tools/cyber_launch/cyber_launch
	

6. cyber_launch:
	
	def main():
    """ main function """
    cyber_path = os.getenv('CYBER_PATH')
    if cyber_path == None:
        logger.error('Error: environment variable CYBER_PATH not found, set environment first.')
        sys.exit(1)
    os.chdir(cyber_path)
    parser = argparse.ArgumentParser(description='cyber launcher')
    subparsers = parser.add_subparsers(help='sub-command help')

    start_parser = subparsers.add_parser('start', help='launch/benchmark.launch')
    start_parser.add_argument('file', nargs='?', action='store', help='launch file, default is cyber.launch')

    stop_parser = subparsers.add_parser('stop', help='stop all the module in launch file')
    stop_parser.add_argument('file', nargs='?', action='store', help='launch file, default stop all the launcher')

    #restart_parser = subparsers.add_parser('restart', help='restart the module')
    #restart_parser.add_argument('file', nargs='?', action='store', help='launch file, default is cyber.launch')

    params = parser.parse_args(sys.argv[1:])

    command = sys.argv[1]
    if command == 'start':
        start(params.file)
    elif command == 'stop':
        stop_launch(params.file)
    #elif command == 'restart':
    #    restart(params.file)
    else:
        logger.error('Invalid command %s' % command)
        sys.exit(1)
	
	
	
	进行一些命令行参数解析，然后调用start(/apollo/modules/dreamview/launch/dreamview.launch)函数启动dreamview模块
	
	def start(launch_file = ''):{
		
		
	}
	
	查看start函数，该函数内容很长，不再详细解释，其主要功能是解析XML文件/apollo/modules/dreamview/launch/dreamview.launch中的各项元素：name、dag_conf、type、process_name、exception_handler，
	其值分别为：dreamview、null、binary、/apollo/bazel-bin/modules/dreamview/dreamview --flagfile=/apollo/modules/common/data/global_flagfile.txt、respawn，
	然后调用ProcessWrapper(process_name.split()[0], 0, [""], process_name, process_type, exception_handler)创建一个ProcessWrapper对象pw，然后调用pw.start()函数启动dreamview模块：

	
7. cyber_launch:	
	
	查看ProcessWrapper类里的start函数：
    def start(self):{
		
	}
	
	在该函数内部调用/apollo/bazel-bin/modules/dreamview/dreamview --flagfile=/apollo/modules/common/data/global_flagfile.txt最终启动了dreamview进程
	

8. dreamview进程的main函数位于/apollo/modules/dreamview/backend/main.cc中
	
	int main(int argc, char *argv[]) {
	  google::ParseCommandLineFlags(&argc, &argv, true);
	  // add by caros for dv performance improve
	  apollo::cyber::GlobalData::Instance()->SetProcessGroup("dreamview_sched");
	  apollo::cyber::Init(argv[0]);

	  apollo::dreamview::Dreamview dreamview;
	  const bool init_success = dreamview.Init().ok() && dreamview.Start().ok();
	  if (!init_success) {
		AERROR << "Failed to initialize dreamview server";
		return -1;
	  }
	  apollo::cyber::WaitForShutdown();
	  dreamview.Stop();
	  apollo::cyber::Clear();
	  return 0;
	}
		
	
	该函数初始化Cyber环境，并调用Dreamview::Init()和Dreamview::Start()函数，启动Dreamview后台监护进程。然后进入消息处理循环，直到等待cyber::WaitForShutdown()返回，清理资源并退出main函数
	
	
	



// 功能模块（以Planning为例）启动过程

Apollo 3.5使用Cyber启动Localization、Perception、Prediction、Planning、Control等功能模块。若只看各模块的BUILD文件，保证你无法找到该模块的启动入口main函数（Apollo 3.5之前的版本均是如此处理）。
下面以Planning模块为例具体阐述:

 1. Planning模块BUILD文件中生成binary文件的配置项如下：
	
	cc_binary(
		name = "libplanning_component.so",
		linkshared = True,
		linkstatic = False,
		deps = [":planning_component_lib"],
	)


在srcs文件planning_component.cc以及deps文件中均找不到main函数。那么main函数被隐藏在哪里？如果没有main函数，binary文件libplanning_component.so又是如何启动的？
答案很简单，planning模块的binary文件libplanning_component.so作为cyber的一个组件启动，不需要main函数。



2. DreamView界面中启动Planning模块的过程。DreamView前端界面操作此处不表，后端的消息响应函数HMI::RegisterMessageHandlers()位于/apollo/modules/dreamview/backend/hmi/hmi.cc文件中
	
	void HMI::RegisterMessageHandlers() {
	}
	

	HMIAction_Parse(action, &hmi_action)用于解析动作参数
	hmi_worker_->Trigger(hmi_action, value)用于执行相关动作。对于Planning模块的启动而言，hmi_action的值为HMIAction::START_MODULE，value的值为Planning
	
	DreamView将操作模式分为多种hmi mode，这些模式位于目录/apollo/modules/dreamview/conf/hmi_modes，每一个配置文件均对应一种hmi mode（更多关于hmi mode的介绍，请参见博客Apollo 3.5 Cyber - 如何為Dreamview新增hmi mode）
	https://blog.csdn.net/weixin_44450715/article/details/86593962

	
	Standard Mode与Navigation Mode对应的dag_files不一样，Standard Mode的dag_files为/apollo/modules/planning/dag/planning.dag，Navigation Mode的dag_files为/apollo/modules/planning/dag/planning_navi.dag。

	HMIWorker::Trigger(const HMIAction action, const std::string& value)函数位于文件/apollo/modules/dreamview/backend/hmi/hmi_worker.cc中


3. hmi_worker.cc:
	bool HMIWorker::Trigger(const HMIAction action, const std::string& value) {
	  AINFO << "HMIAction " << HMIAction_Name(action) << "(" << value
			<< ") was triggered!";
	  switch (action) {
		case HMIAction::CHANGE_MODE:
		  ChangeMode(value);
		  break;
		case HMIAction::CHANGE_MAP:
		  ChangeMap(value);
		  break;
		case HMIAction::CHANGE_VEHICLE:
		  ChangeVehicle(value);
		  break;
		case HMIAction::START_MODULE:
		  StartModule(value);
		  break;
		case HMIAction::STOP_MODULE:
		  StopModule(value);
		  break;
		case HMIAction::RECORD_AUDIO:
		  RecordAudio(value);
		  break;
		default:
		  AERROR << "HMIAction not implemented, yet!";
		  return false;
	  }
	  return true;
	}




4. hmi_worker.cc:
	继续研究HMIWorker::StartModule(const std::string& module)函数,函数中成员变量current_mode_保存着当前hmi mode对应配置文件包含的所有配置项
	例如modules/dreamview/conf/hmi_modes/mkz_standard_debug.pb.txt里面就包含了MKZ标准调试模式下所有的功能模块，该配置文件通过HMIWorker::LoadMode(const std::string& mode_config_path)函数读入到成员变量current_mode_中
    如果基于字符串module查找到了对应的模块名以及对应的启动配置文件dag_files，则调用System函数（内部实际调用std::system函数)基于命令module_conf->start_command()启动一个进程。
	这个start_command从何而来？需进一步分析HMIWorker::LoadMode(const std::string& mode_config_path)函数

	
	
     void HMIWorker::StartModule(const std::string& module) const {
	  const Module* module_conf = FindOrNull(current_mode_.modules(), module);
	  if (module_conf != nullptr) {
		System(module_conf->start_command());
	  } else {
		AERROR << "Cannot find module " << module;
	  }
	}


5. hmi_worker.cc

	HMIMode HMIWorker::LoadMode(const std::string& mode_config_path) {
		
	}

	
	构建出的start_command格式为nohup mainboard -p <process_group> -d <dag> ... &，其中，<process_group>与dag均来自于当前hmi mode对应的配置文件
	
	以modules/dreamview/conf/hmi_modes/mkz_close_loop.pb.txt为例，它包含两个cyber_modules配置项，对于Computer模块而言，它包含了11个dag_files文件（对应11个子功能模块），这些子功能模块全部属于名为compute_sched的process_group。

	process_group就是Cyber中调度配置文件scheduler conf的名字，process_group: "compute_sched"表明使用配置文件cyber/conf/compute_sched.conf进行任务调度，process_group: "control_sched"表明使用配置文件control_sched.conf进行任务调度

   <dag>自不必言，表示一个DAG（Directed Acyclic Graph，有向无环图）节点，每个子功能模块对应一个dag_files，Planning子功能模块对应的dag_files为/apollo/modules/planning/dag/planning.dag



6. nohup mainboard -p compute_sched -d /apollo/modules/planning/dag/planning.dag &

	nohup表示非挂断方式启动，mainboard无疑是启动的主程序，入口main函数必定包含于其中。process_group的意义不是那么大，无非对功能模块分组而已；dag_files才是我们启动相关功能模块的真正配置文件


7. 查看cyber模块的构建文件/apollo/cyber/BUILD,可执行文件mainboard的踪迹水落石出。果不其然，入口函数main位于文件cyber/mainboard/mainboard.cc中.


8. mainboard.cc:

  int main(int argc, char** argv) {
	  google::SetUsageMessage("we use this program to load dag and run user apps.");

	  // parse the argument
	  ModuleArgument module_args;
	  module_args.ParseArgument(argc, argv);

	  // initialize cyber
	  apollo::cyber::Init(argv[0]);

	  // start module
	  ModuleController controller(module_args);
	  if (!controller.Init()) {
		controller.Clear();
		AERROR << "module start error.";
		return -1;
	  }

	  apollo::cyber::WaitForShutdown();
	  controller.Clear();
	  AINFO << "exit mainboard.";

	  return 0;
	}


   main函数十分简单，首先是解析参数，初始化cyber环境，接下来创建一个ModuleController类对象controller，之后调用controller.Init()启动相关功能模块。最后，一直等待cyber::WaitForShutdown()返回，清理资源并退出main函数。
   ModuleController::Init()函数十分简单，内部调用了ModuleController::LoadAll()函数



9. module_controller.cc

	bool ModuleController::LoadAll() {
		
	}
	
	
	上述函数处理一个dag_conf配置文件循环，读取配置文件中的所有dag_conf，并逐一调用bool ModuleController::LoadModule(const std::string& path)函数加载功能模块
	
	上述函数从磁盘配置文件读取配置信息，并调用bool ModuleController::LoadModule(const DagConfig& dag_config)函数加载功能模块
	
	
	
	
// Planning模块作为Cyber组件注册并动态创建的过程

整个Planning模块的启动过程已阐述完毕，但仍有一个问题需要解决：Planning模块是如何作为Cyber的一个组件注册并动态创建的？
	
1. planning_component.h 

 class PlanningComponent final : public cyber::Component<prediction::PredictionObstacles, canbus::Chassis,localization::LocalizationEstimate> {	
							  
																																			}
	modules/planning/planning_component.h的组件类PlanningComponent继承自cyber::Component<prediction::PredictionObstacles, canbus::Chassis, localization::LocalizationEstimate>，里面管理着PlanningBase类对象指针
   （Apollo 3.5基于场景概念进行规划，目前从PlanningBase类派生出三个规划类：StdPlanning（高精地图模式）、NaviPlanning（实时相对地图模式）、OpenSpacePlanning（自由空间模式），可通过目录modules/planning/dag下的配置文件指定选用何种场景）

2. 使用宏CYBER_REGISTER_COMPONENT(PlanningComponent)将规划组件PlanningComponent注册到Cyber的组件类管理器	
	apollo/cyber/component/component.h:
	#define CYBER_REGISTER_COMPONENT(name) \
	CLASS_LOADER_REGISTER_CLASS(name, apollo::cyber::ComponentBase)
	
	
	apollo/cyber/class_loader/class_loader_register_macro.h:
	// register class macro
	#define CLASS_LOADER_REGISTER_CLASS(Derived, Base) \
	  CLASS_LOADER_REGISTER_CLASS_INTERNAL_1(Derived, Base, __COUNTER__)

	
	apollo/cyber/class_loader/class_loader_register_macro.h:
	
	#define CLASS_LOADER_REGISTER_CLASS_INTERNAL_1(Derived, Base, UniqueID) \
	CLASS_LOADER_REGISTER_CLASS_INTERNAL(Derived, Base, UniqueID)
	
	
	apollo/cyber/class_loader/class_loader_register_macro.h:
	#define CLASS_LOADER_REGISTER_CLASS_INTERNAL(Derived, Base, UniqueID)     \
	  namespace {                                                             \
	  struct ProxyType##UniqueID {                                            \
		ProxyType##UniqueID() {                                               \
		  apollo::cyber::class_loader::utility::RegisterClass<Derived, Base>( \
			  #Derived, #Base);                                               \
		}                                                                     \
	  };                                                                      \
	  static ProxyType##UniqueID g_register_class_##UniqueID;                 \
	  }
		
	
	注意两点：第一，上述定义位于namespace apollo::planning内；第二，___COUNTER__是C语言的一个计数器宏，这里仅代表一个占位符，实际展开时可能就是78之类的数字，亦即ProxyType__COUNTER__实际上应为ProxyType78之类的命名。上述代码简洁明了，首先定义一个结构体ProxyType__COUNTER__，该结构体仅包含一个构造函数，在内部调用apollo::cyber::class_loader::utility::RegisterClass<PlanningComponent, apollo::cyber::ComponentBase>注册apollo::cyber::ComponentBase类的派生类PlanningComponent。并定义一个静态全局结构体ProxyType__COUNTER__变量：g_register_class___COUNTER__。
    继续观察apollo::cyber::class_loader::utility::RegisterClass函数：


3. apollo/cyber/class_loader/utility/class_loader_utility.h

	template <typename Derived, typename Base>
	void RegisterClass(const std::string& class_name, const std::string& base_class_name) {
	  AINFO << "registerclass:" << class_name << "," << base_class_name << "," << GetCurLoadingLibraryName();
	  utility::AbstractClassFactory<Base>* new_class_factrory_obj = new utility::ClassFactory<Derived, Base>(class_name, base_class_name);  //创建一个模板类utility::ClassFactory<Derived, Base>对象new_class_factrory_obj
	  new_class_factrory_obj->AddOwnedClassLoader(GetCurActiveClassLoader());      //添加类加载器
	  new_class_factrory_obj->SetRelativeLibraryPath(GetCurLoadingLibraryName());  //设置加载库的路径

	  GetClassFactoryMapMapMutex().lock();
	  ClassClassFactoryMap& factory_map =  GetClassFactoryMapByBaseClass(typeid(Base).name());
	  factory_map[class_name] = new_class_factrory_obj;  //最后将工厂类对象加入到ClassClassFactoryMap对象factory_map统一管理
	  GetClassFactoryMapMapMutex().unlock();
	}
		
		
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	