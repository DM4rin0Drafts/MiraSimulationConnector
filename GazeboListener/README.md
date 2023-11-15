# Start GazeboListener

This unit listen to the defined channels given the gazebo simulation and publishes the values in 
miracenter. 

To start the gazebo simulation channel listener, you need to run `miracenter` with `-p <port>`. In addition, you need to run the `GazeboListener` with `-k <port>`. Example:

    - ./domains/simulation/GazeboListener/bin/gazebolistener -k 12345

The port for configuring miracenter and the `GazeboListener` needs to be the same.
    
