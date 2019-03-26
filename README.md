# vrep-python-test

Purpose :
Joint space Torque control
Operational space Torque control (only position)


1. Download and extract recent V-REP PRO/DEU (Hereafter, vrep folder)
2. Git clone this repository (Hereafter, source folder)
3. In vrep/programming/bluezero

   ```mkdir build && cd build```

   ```cmake .. -DCMAKE_BUILD_TYPE=RELEASE```
   
   ```make```
   
   After compiling ti, copy libb0.s0 in build folder and paste it into source/Extern 
   
 4. Copy b0.py and b0RemoteApi.py in vrep/programming/b0RemoteApiBindings/python/python
 
    And Paste them into source/Extern
    
 5. Open ur5_torque.ttt file with Vrep, for example
 
   ```sudo $HOME/vrep/vrep.sh $Home/source/ur5_torque.ttt```
   
   and Turn on the Bluezero server
   
   in top-menu bar (Add-ons -> b0remoteApiSever click)
   
 6. Run demo script
   
 
 
 # TODO
 Bug#1: to exit the python script, push Ctrl+c
 
