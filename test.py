from .dm_imu import DM_IMU_USB
import matplotlib.pyplot as plt
instance=DM_IMU_USB()
is_plot=True
plot_limit=300
if is_plot:
    instance.start("COM3")
    plt.subplots_adjust(hspace=0.8)
    #plt.figure(figsize=(10,20))
    fig_acc_x=[0]*100
    fig_acc_y=[0]*100
    fig_acc_z=[0]*100
    fig_agv_x=[0]*100
    fig_agv_y=[0]*100
    fig_agv_z=[0]*100
    fig_els_x=[0]*100
    fig_els_y=[0]*100
    fig_els_z=[0]*100
    x=[i for i in range(1,101)]
    plt.ion()
    while plot_limit>0:
        plt.clf()
        data=instance.get_imudata()
        print("| limit loop:",plot_limit,"; current data:",data)
        fig_acc_x.pop(0)
        fig_acc_x.append(data[0])
        fig_acc_y.pop(0)
        fig_acc_y.append(data[1])
        fig_acc_z.pop(0)
        fig_acc_z.append(data[2])
        fig_agv_x.pop(0)
        fig_agv_x.append(data[3])
        fig_agv_y.pop(0)
        fig_agv_y.append(data[4])
        fig_agv_z.pop(0)
        fig_agv_z.append(data[5])
        fig_els_x.pop(0)
        fig_els_x.append(data[6])
        fig_els_y.pop(0)
        fig_els_y.append(data[7])
        fig_els_z.pop(0)
        fig_els_z.append(data[8])
        plt.subplot(3,1,1)
        plt.title('Acceleration (m/s^2)')
        plt.plot(x,fig_acc_x,'r',label='x_axis')
        plt.plot(x,fig_acc_y,'g',label='y_axis')
        plt.plot(x,fig_acc_z,'b',label='z_axis')
        plt.subplot(3,1,2)
        plt.title('Angular velocity (rad/s)')
        plt.plot(x,fig_agv_x,'r',label='x_axis')
        plt.plot(x,fig_agv_y,'g',label='y_axis')
        plt.plot(x,fig_agv_z,'b',label='z_axis')
        plt.subplot(3,1,3)
        plt.title('Euler angles (°)')
        plt.plot(x,fig_els_x,'r',label='x_axis')
        plt.plot(x,fig_els_y,'g',label='y_axis')
        plt.plot(x,fig_els_z,'b',label='z_axis')
        plot_limit-=1
        plt.pause(0.0001)
    plt.ioff()
    instance.stop()