add_library(CybertronUnrealNodeBase SHARED "")

include_directories("${CYBERTRON_PATH_ROOT}/3rdparty/asio")
include_directories("${CYBERTRON_PATH_ROOT}/3rdparty")
include_directories("${CYBERTRON_PATH_ROOT}/3rdparty/gtest/include")
include_directories("${CYBERTRON_PATH_ROOT}/3rdparty/glfw-3.2.1/include")
include_directories("${CYBERTRON_PATH_ROOT}/3rdparty/glfw-3.2.1/deps")
 
include_directories("${CYBERTRON_PATH_FOUNDATION}/Source/CybertronProtocol/Output/cpp")
include_directories("${CYBERTRON_PATH_FOUNDATION}/Source/CybertronCore/include")

IF (WIN32)
	add_definitions(-D_WIN32_WINNT=0x0501)
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi")
	set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DEBUG /OPT:REF /OPT:ICF")
ENDIF()

add_definitions(-DNOMINMAX)
add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)
add_definitions(-DBUILD_UNREAL_BRIDGE)

foreach (path Graphics)
	file(GLOB_RECURSE srcs ${path}/*.h ${path}/*.hh ${path}/*i ${path}/*.hpp ${path}/*.c ${path}/*.cc ${path}/*.cpp)
	target_sources(CybertronUnrealNodeBase PRIVATE ${srcs})
	source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${srcs})
endforeach (path)

file(GLOB srcs *.h *.hh *i *.hpp *.c *.cc *.cpp)
target_sources(CybertronUnrealNodeBase PRIVATE ${srcs})
source_group(Source FILES ${srcs})

SetDefaultTargetProperties(CybertronUnrealNodeBase)

include_directories("${CYBERTRON_PATH_ROOT}/3rdparty/artifactory/ffmpeg/include")
set(FFMPEG_BIN_PATH ${CYBERTRON_PATH_ROOT}/3rdparty/artifactory/ffmpeg/bin)
set(FFMPEG_LIB_PATH ${CYBERTRON_PATH_ROOT}/3rdparty/artifactory/ffmpeg/lib)
set(CURL_LIB_PATH ${CYBERTRON_PATH_ROOT}/3rdparty/libcurl)
IF (WIN32)
    SET(FOUNDATION_LIB_PATH ${CYBERTRON_PATH_FOUNDATION}/Build/build_vs2015/lib)
    target_link_libraries(CybertronUnrealNodeBase debug ${FOUNDATION_LIB_PATH}/Debug/CybertronCore.lib)
    target_link_libraries(CybertronUnrealNodeBase debug ${FOUNDATION_LIB_PATH}/Debug/protobuf.lib)
    
    target_link_libraries(CybertronUnrealNodeBase optimized ${FOUNDATION_LIB_PATH}/Release/CybertronCore.lib)
    target_link_libraries(CybertronUnrealNodeBase optimized ${FOUNDATION_LIB_PATH}/Release/protobuf.lib)
    
    target_link_libraries(CybertronUnrealNodeBase ${FFMPEG_LIB_PATH}/avcodec.lib)
    target_link_libraries(CybertronUnrealNodeBase ${FFMPEG_LIB_PATH}/avformat.lib)
    target_link_libraries(CybertronUnrealNodeBase ${FFMPEG_LIB_PATH}/avutil.lib)
    target_link_libraries(CybertronUnrealNodeBase ${FFMPEG_LIB_PATH}/swscale.lib)
    target_link_libraries(CybertronUnrealNodeBase ${FFMPEG_LIB_PATH}/swresample.lib)

	# for curl static lib
	target_link_libraries(CybertronUnrealNodeBase
       ws2_32
       crypt32
       Wldap32
       Normaliz
    )
	target_link_libraries(CybertronUnrealNodeBase ${CURL_LIB_PATH}/lib_win_x64/libcurl_a.lib)
	
	add_custom_command(TARGET CybertronUnrealNodeBase 
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/bin/CybertronUnrealNodeBase.dll ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/Vehicle51Sim/Binaries/Win64/CybertronUnrealNodeBase.dll
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/bin/CybertronUnrealNodeBase.pdb ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/Vehicle51Sim/Binaries/Win64/CybertronUnrealNodeBase.pdb
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_BIN_PATH}/avcodec-58.dll ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/Vehicle51Sim/Binaries/Win64/avcodec-58.dll
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_BIN_PATH}/avformat-58.dll ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/Vehicle51Sim/Binaries/Win64/avformat-58.dll
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_BIN_PATH}/avutil-56.dll ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/Vehicle51Sim/Binaries/Win64/avutil-56.dll
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_BIN_PATH}/swresample-3.dll ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/Vehicle51Sim/Binaries/Win64/swresample-3.dll
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_BIN_PATH}/swscale-5.dll ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/Vehicle51Sim/Binaries/Win64/swscale-5.dll
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/bin/CybertronUnrealNodeBase.dll ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/SUMO4UE/Binaries/Win64/CybertronUnrealNodeBase.dll
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/bin/CybertronUnrealNodeBase.pdb ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/SUMO4UE/Binaries/Win64/CybertronUnrealNodeBase.pdb
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_BIN_PATH}/avcodec-58.dll ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/SUMO4UE/Binaries/Win64/avcodec-58.dll
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_BIN_PATH}/avformat-58.dll ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/SUMO4UE/Binaries/Win64/avformat-58.dll
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_BIN_PATH}/avutil-56.dll ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/SUMO4UE/Binaries/Win64/avutil-56.dll
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_BIN_PATH}/swresample-3.dll ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/SUMO4UE/Binaries/Win64/swresample-3.dll
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_BIN_PATH}/swscale-5.dll ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/SUMO4UE/Binaries/Win64/swscale-5.dll
    )
ENDIF()

IF (UNIX)
    SET(FOUNDATION_DEBUG_LIB_PATH ${CYBERTRON_PATH_FOUNDATION}/Build/build_debug/lib)
    SET(FOUNDATION_RELEASE_LIB_PATH ${CYBERTRON_PATH_FOUNDATION}/Build/build_release/lib)
    target_link_libraries(CybertronUnrealNodeBase debug ${FOUNDATION_DEBUG_LIB_PATH}/libCybertronCore.a)
    target_link_libraries(CybertronUnrealNodeBase debug ${FOUNDATION_DEBUG_LIB_PATH}/libprotobuf.a)
    
    target_link_libraries(CybertronUnrealNodeBase optimized ${FOUNDATION_RELEASE_LIB_PATH}/libCybertronCore.a)
    target_link_libraries(CybertronUnrealNodeBase optimized ${FOUNDATION_RELEASE_LIB_PATH}/libprotobuf.a)
    
    target_link_libraries(CybertronUnrealNodeBase ${FFMPEG_LIB_PATH}/libavcodec.so.58)
    target_link_libraries(CybertronUnrealNodeBase ${FFMPEG_LIB_PATH}/libavformat.so.58)
    target_link_libraries(CybertronUnrealNodeBase ${FFMPEG_LIB_PATH}/libavutil.so.56)
    target_link_libraries(CybertronUnrealNodeBase ${FFMPEG_LIB_PATH}/libswscale.so.5)
    target_link_libraries(CybertronUnrealNodeBase ${FFMPEG_LIB_PATH}/libswresample.so.3)
    target_link_libraries(CybertronUnrealNodeBase ${FFMPEG_LIB_PATH}/libx264.so.148)

    add_custom_command(TARGET CybertronUnrealNodeBase 
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/lib/libCybertronUnrealNodeBase.so ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/Vehicle51Sim/Binaries/Linux/libCybertronUnrealNodeBase.so
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_LIB_PATH}/libavcodec.so.58 ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/Vehicle51Sim/Binaries/Linux/libavcodec.so.58
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_LIB_PATH}/libavformat.so.58 ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/Vehicle51Sim/Binaries/Linux/libavformat.so.58
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_LIB_PATH}/libavutil.so.56 ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/Vehicle51Sim/Binaries/Linux/libavutil.so.56
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_LIB_PATH}/libswscale.so.5 ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/Vehicle51Sim/Binaries/Linux/libswscale.so.5
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_LIB_PATH}/libswresample.so.3 ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/Vehicle51Sim/Binaries/Linux/libswresample.so.3
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_LIB_PATH}/libx264.so.148 ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/Vehicle51Sim/Binaries/Linux/libx264.so.148
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/lib/libCybertronUnrealNodeBase.so ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/SUMO4UE/Binaries/Linux/libCybertronUnrealNodeBase.so
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_LIB_PATH}/libavcodec.so.58 ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/SUMO4UE/Binaries/Linux/libavcodec.so.58
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_LIB_PATH}/libavformat.so.58 ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/SUMO4UE/Binaries/Linux/libavformat.so.58
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_LIB_PATH}/libavutil.so.56 ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/SUMO4UE/Binaries/Linux/libavutil.so.56
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_LIB_PATH}/libswscale.so.5 ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/SUMO4UE/Binaries/Linux/libswscale.so.5
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_LIB_PATH}/libswresample.so.3 ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/SUMO4UE/Binaries/Linux/libswresample.so.3
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FFMPEG_LIB_PATH}/libx264.so.148 ${CYBERTRON_PATH_ROOT}/Modules/Unreal/Engine/Plugins/51HiTech/SUMO4UE/Binaries/Linux/libx264.so.148
    )
ENDIF()
