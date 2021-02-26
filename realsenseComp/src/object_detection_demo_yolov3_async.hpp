// Copyright (C) 2018-2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

///////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <string>
#include <vector>
#include <gflags/gflags.h>
#include <iostream>


static const char help_message[] = "Print a usage message.";
static const char video_message[] = "Required. Path to a video file (specify \"cam\" to work with camera).";
static const char model_message[] = "Required. Path to an .xml file with a trained model.";
static const char target_device_message[] = "Optional. Specify a target device to infer on (the list of available devices is shown below). "
                                            "Default value is CPU. The demo will look for a suitable plugin for the specified device";
static const char performance_counter_message[] = "Optional. Enable per-layer performance report.";
static const char custom_cldnn_message[] = "Optional. Required for GPU custom kernels. "
                                           "Absolute path to the .xml file with the kernels description.";
static const char custom_cpu_library_message[] = "Optional. Required for CPU custom layers. "
                                                 "Absolute path to a shared library with the layers implementation.";
static const char thresh_output_message[] = "Optional. Probability threshold for detections.";
static const char iou_thresh_output_message[] = "Optional. Filtering intersection over union threshold for overlapping boxes.";
static const char raw_output_message[] = "Optional. Output inference results raw values showing.";
static const char input_resizable_message[] = "Optional. Enable resizable input with support of ROI crop and auto resize.";
static const char no_show_processed_video[] = "Optional. Do not show processed video.";
static const char utilization_monitors_message[] = "Optional. List of monitors to show initially.";


DEFINE_bool(h, false, help_message);
DEFINE_string(i, "", video_message);
DEFINE_string(m, "./Tiny_Yolo_Linear/1frozen_darknet_yolov3_model.xml", model_message);
DEFINE_string(d, "CPU", target_device_message);
DEFINE_bool(pc, false, performance_counter_message);
DEFINE_string(c, "", custom_cldnn_message);
DEFINE_string(l, "", custom_cpu_library_message);
DEFINE_bool(r, false, raw_output_message);
DEFINE_double(t, 0.5, thresh_output_message);
DEFINE_double(iou_t, 0.4, iou_thresh_output_message);
DEFINE_bool(auto_resize, false, input_resizable_message);
DEFINE_bool(no_show, false, no_show_processed_video);
DEFINE_string(u, "", utilization_monitors_message);

