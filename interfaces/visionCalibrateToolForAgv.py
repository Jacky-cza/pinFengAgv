import rtde_receive

rtdeReceive = rtde_receive.RTDEReceiveInterface("192.168.123.238")
print(rtdeReceive.getActualTCPPose())

"""
目前点找数据方法：
1.AGV工作站点先在地图上定好，不能距离伸出的窗口太近也不能太远
2.AGV到达工作站点之后调整相机位置，使拍到的2个mark点尽量在图片正中心不偏不倚尽量不旋转(小用水平仪测量)，上下左右留一些容差的距离，目的是使相机的镜头中心尽量放在两个mark点中间位置
3.在该位置使用10.017棋格板标定相机，转成mm，调整相机的曝光亮度等参数，记得及时保存；
4.标定好之后此时该拍照位置就是零位，零位时候默认相机的镜头中心点与2个mark点之间的那个中心点重合的，拿
    set 1to1:
    此时poseTranse()里面第二个参数的偏移列表[x,y,z,rx,ry,rz]
    x：+0.09(CAD测量得出，该数值是在tool坐标系下面的偏移)，
    y：-0.26, 0.13325+0.1273=0.260，在tool坐标系下面为负值即：
    z的数值可以在夹爪上放上试管框，判断从标定的零位高度下降到传递窗对应的槽上方一些槽口的位置(不是槽底)
    
    将上步得到的[x,y,z,0,0,0]作为参数先写入，通过 pose=robotControlClient.poseTrans(URpose,[x,y,z,0,0,0]),moveJ_IK或者moveL
    驱使夹爪上面带试管框的UR运动到目标位置，此时可根据运动的实际情况调整x,y,z再调整rx,ry,rz,因为实际情况下x,y,rz的数值跟0.09,-0.26,0肯定是有出入的，
    主要是因为第2步标定的时候，相机是肉眼看的大概在2个mark正中心但是实际上肯定有出入，rz有出入是因为rx和ry可以通过小水平仪查看，但是rz看不出来
    先调整xyz到槽的位置后,再调整rx,ry,rz,而且rx,ry,rz的调整不会很大，标定还算好的情况下基本上在0.5-2°之间，可以将框放在槽内，这样rx,ry,rz需要调整多少比较方便看
    
    [x,y,z,rx,ry,rz]调整好之后，配置set步骤的时候，在相应的UR程序里面夹爪在移动到调整好的位置之后需要下降多少才能放置好试管框，此高度是UR运动到[x,y,z,rx,ry,rz]之后记下来Z数值，
    示教器将框下降到槽的底部，再下降使夹爪下降到方便退出来的位置再记下来Z的数值，相减就能得到Z的差值，写在UR程序里面；
    
    set：1to2   
    x：-0.09(CAD测量得出，该数值是在tool坐标系下面的偏移所以是负值)，
    y：-0.26, 0.13325+0.1273=0.260，在tool坐标系下面为负值即：
    其它同1to1
    
    get: 1to1
    采用set 1to1 时候标定好的数据，对应的UR程序里面UR下降多少高度可以抓取到试管框跟set 1to1时候UR程序里面的一样
    
    get: 2to1
    采用set 1to2 时候标定好的数据，对应的UR程序里面UR下降多少高度可以抓取到试管框跟set 1to2时候UR程序里面的一样


拍照获得数据比较快，根据拍照数据计算出位姿然后驱使UR动作这一步有点慢，后面可以改成第一拍照然后驱使UR夹爪角度转正，
第二次拍照完成后不用驱使夹爪纠偏计算好目标点位姿就好，之前第三部是完全纠偏之后拍照获取数据记录到log的，也可以去掉，这样整体速度会快不少



"""
