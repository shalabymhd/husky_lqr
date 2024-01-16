#include "Lqr.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "lqr");

    Lqr lqr_controller;
    
    // TODO: Add visualization code here

    ros::spin();

    return 0;
}