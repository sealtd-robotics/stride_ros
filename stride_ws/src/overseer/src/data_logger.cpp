// ========================================================================
// Copyright (c) 2022, SEA Ltd.
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 
// ========================================================================

#include <ros/ros.h>
#include <subscribers.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "data logger");
    ros::NodeHandle nh;
    int r = 50;
    ros::param::get("~record_rate", r);

    ros::Rate rate(r);

    DataRecorderSub ds(&nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok()) {
        if(ds.GetRecordingStatus()) {
            ds.WriteBinary();
        }
        else if (!ds.GetRecordingStatus() && !ds.GetConvertToCsvStatus()) {
            ds.ConvertBin2Csv();
            ds.UpdateConvertToCsvStatus(true);
        }
        // ros::spinOnce();
        rate.sleep();
    }

    return 0;
}