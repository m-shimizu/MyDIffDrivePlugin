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
You can add below codes into your model.sdf to use your MyDIffDrivePlugin.

'''
    <plugin filename="libDiffDrivePlugin.so" name="diff_drive">
      <left_joint>left_wheel_hinge</left_joint>
      <right_joint>right_wheel_hinge</right_joint>
      <shoulderTAG>shoulder</shoulderTAG>
      <torque>5</torque>
    </plugin>
'''

Edited: 12 July 2018
