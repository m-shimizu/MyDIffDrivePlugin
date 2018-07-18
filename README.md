# MyDIffDrivePlugin  
This is a sample program for my class.  

## How to use.  
Download this repository from the github.  

    $ git clone https://github.com/m-shimizu/MyDIffDrivePlugin  

Build the program.  

    $ cd MyDIffDrivePlugin  
    $ mkdir build  
    $ cd build  
    $ cmake ..  
    $ make  

## Add the plugin tag in your model.sdf.  
You can add below codes after <joint> tags in your model.sdf to use your MyDIffDrivePlugin.

```
    <plugin filename="libMyDiffDrivePlugin.so" name="diff_drive">  
      <left_joint>left_wheel_hinge</left_joint>  
      <right_joint>right_wheel_hinge</right_joint>  
      <shoulderTAG>shoulder</shoulderTAG>  
      <torque>5</torque>  
    </plugin>  
```

## How to spawn your robot  
You can spawn your robot by below command.  

    $ cd MyDIffDrivePlugin  
    $ . setup.bash  
    $ gazebo myrobot.world  

## How to control your robot
You can control your robot , press keys(q,a,z,e,d,c) focused on the Gazebo window.  
This is realized by using libKeyboardGUIPlugin.so.  
You can find more details by visiting [here](https://bitbucket.org/osrf/gazebo/pull-requests/2652/added-support-for-tracked-vehicles/diff) and [here](https://github.com/osrf/car_demo/issues/25).  

Edited: 18 July 2018
