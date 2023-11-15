
#include <iostream>

int main(int argc, char **argv) {

    for (int i = 0; i< 50; i++) {
        std::cout << "Publisher: " << i << std::endl;
    }

    /* cmd_vel to controll robot in simulation
    rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"
    */

    return 0;
}