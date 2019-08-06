 人工发送prediction::PredictionObstacles消息
为提高消息处理的实时性和灵活性，Apollo 3.5的Planning模块不再基于定时器触发更新，而是基于三个输入消息的改变而动态更新，这三个输入消息分别为：prediction::PredictionObstacles、canbus::Chassis、localization::LocalizationEstimate。也就是说，只有上述三个消息同时存在时，Planning模块的消息处理函数PlanningComponent::Proc()才会被调用，而具体的某一类规划算法（例如OnLanePlanning）才会真正工作。

若某条消息因为特殊原因不能及时发送，解决办法就是人工生成假消息。例如，若不能收到prediction::PredictionObstacles消息，则可在在Docker内部通过如下命令生成假prediction::PredictionObstacles消息：

cyber_launch start /apollo/modules/tools/prediction/fake_prediction/fake_prediction.launch
1
该假消息的具体生成代码见/apollo/modules/tools/prediction/fake_prediction，其他假消息的生成可参照该示例撰写。

3.3.2 人工发送perception::TrafficLightDetection消息
调试规划算法时，需要动态改变红绿灯的信号状态，可以通过如下命令人工发送perception::TrafficLightDetection消息来实现：

cyber_launch start /apollo/modules/tools/manual_traffic_light/manual_traffic_light.launch



