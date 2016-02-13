
Fuses data from VSLAM Sonar and IMU to provide metric state measurements on the Nayan quadcopter platform. More details on the Nayan quadcopter can be found on [nayan_wiki](http://aus.co.in/wiki/Main_Page)
## Organization: 

This package contains the following modules 

### Position fusion
This performs the main fusion and links up all different units.
[Running Position fusion on OBC](http://aus.co.in/wiki/Sample_Code_2:_Flying_with_SVO%2BSONAR_using_Slave_Processor)
```bash
      roslaunch cv_sens sensor_fusion.launch
```

### Plugins for mavros
A few plugins have been written for mavros which provide extra debug variables on the Onboard computer when connecting to the flight control unit and communicating via mavlink interface
[Communication between Slave and OnBoard Computer](http://aus.co.in/wiki/Interfacing_between_OBC_and_Slave_Processor)
```bash
      roslaunch cv_sens nayan_slave.launch
```

### Sending setpoints to the flight control unit using onboard computer
```bash
rosrun cv_sens setpoint_enu _x:=-1 _y:=-1 _z:=-1
```

### Sending dummy positions to the flight control unit
```bash
rosrun cv_sens dummy_visionpose_publisher
```
