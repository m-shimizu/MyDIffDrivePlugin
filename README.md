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

Edited: 18 July 2018
