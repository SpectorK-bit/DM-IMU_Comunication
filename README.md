# 这是什么？ | What is this?
这个程序用于计算机通过Python与DM-IMU通信，其一，接收并解析来自DM-IMU的数据，其二，向DM-IMU发送指令。
This program is used for computers to communicate with DM-IMU through Python. Firstly, it receives and parses data from DM-IMU, and secondly, it sends instructions to DM-IMU.
# 怎么用？ | How to use it?
详见test.py。
See test.py for details.
# 为什么没有发送指令的示例？ | Why are there no examples of sending instructions?
由于作者的需求只是接收其数据，故未对指令发送部分进行测试，但该部分的数据帧满足DM-IMU手册对数据帧的描述。
Due to the author's requirement of only receiving their data, the instruction sending part was not tested, but the data frames in this part meet the description of data frames in the DM-IMU manual.

# 注意 | Attention
获取的三项数据（加速度、角速度、欧拉角）是异步更新的，也就是说，加速度数据是这一帧的，但角速度数据、欧拉角数据可能是上一帧的。
The three data obtained (acceleration, angular velocity, Euler angle) are updated asynchronously, which means that the acceleration data is from this frame, but the angular velocity data and Euler angle data may be from the previous frame.
