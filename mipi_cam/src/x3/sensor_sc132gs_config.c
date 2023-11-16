// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sensor_sc132gs_config.h"

static const int width = 1088;
static const int height = 1280;

MIPI_SENSOR_INFO_S SENSOR_SC132GS_30FPS_1280P_LINEAR_INFO = {
        .deseEnable = 0,
        .inputMode = INPUT_MODE_MIPI,
        .sensorInfo = {
                .port = 0,
                .dev_port = 0,
                .bus_type = 0,
                .bus_num = 2,
                .fps = 30,
                .resolution = width,
                .sensor_addr = 0x30,
                .entry_index = 0,
                .sensor_mode = NORMAL_M,
                .reg_width = 16,
                .sensor_name = "sc132gs",
                .deserial_index = 0,
                .deserial_port = 0,
                // .gpio_num = 1,
                // .gpio_pin[0] = 119,
                // .gpio_level[0] = 0,
                .extra_mode = 2
        }
};

MIPI_ATTR_S MIPI_SENSOR_SC132GS_30FPS_1280P_LINEAR_ATTR = {
        .mipi_host_cfg =
                {
                        2,    /* lane */
                        0x2b, /* datatype */
                        2400, /* mclk */
                        1200,  /* mipiclk */
                        30,   /* fps */
                        width, /* width  */
                        height, /*height */
                        1400, /* linlength */
                        1500, /* framelength */
                        20,   /* settle */
                        1,    /*chnnal_num*/
                        {0}   /*vc */
                },
        .dev_enable = 0, /*  mipi dev enable */
};

VIN_DEV_ATTR_S DEV_ATTR_SC132GS_LINEAR_BASE = {
        .stSize =
                {
                        0,    /*format*/
                        width, /*width*/
                        height, /*height*/
                        1     /*pix_length*/
                },
        {
                .mipiAttr =
                        {
                                .enable = 1,
                                .ipi_channels = 1,
                                .ipi_mode = 0,
                                .enable_mux_out = 1,
                                .enable_frame_id = 1,
                                .enable_bypass = 0,
                                .enable_line_shift = 0,
                                .enable_id_decoder = 0,
                                .set_init_frame_id = 0,
                                .set_line_shift_count = 0,
                                .set_bypass_channels = 1,
                                .enable_pattern = 0,
                        },
        },
        .DdrIspAttr = {.buf_num = 8,
                .raw_feedback_en = 0,
                .data =
                        {
                                .format = 0,
                                .width = width,
                                .height = height,
                                .pix_length = 1,
                        }},
        .outDdrAttr =
                {
                        .stride = 1360, .buffer_num = 8,
                        // .frameDepth = 0,
                },
        .outIspAttr = {
                .dol_exp_num = 1,
                .enable_dgain = 0,
                .set_dgain_short = 0,
                .set_dgain_medium = 0,
                .set_dgain_long = 0,
                .vc_short_seq = 0,
                .vc_medium_seq = 0,
                .vc_long_seq = 0,
        }
};

VIN_PIPE_ATTR_S PIPE_ATTR_SC132GS_LINEAR_BASE = {
        .ddrOutBufNum = 8,
        .snsMode = SENSOR_NORMAL_MODE,
        .stSize =
                {
                        .format = 0,
                        .width = width,
                        .height = height,
                },
        .cfaPattern = PIPE_BAYER_BGGR,
        .temperMode = 2,
        .ispBypassEn = 0,
        .ispAlgoState = 1,
        .bitwidth = 10,
        .calib = {
                .mode = 1,
                .lname = "/lib/sensorlib/libsc132gs_linear.so",
        }
};

VIN_DIS_ATTR_S DIS_ATTR_SC132GS_BASE = {
        .picSize =
                {
                        .pic_w = width - 1,
                        .pic_h = height - 1,
                },
        .disPath =
                {
                        .rg_dis_enable = 0,
                        .rg_dis_path_sel = 1,
                },
        .disHratio = 65536,
        .disVratio = 65536,
        .xCrop =
                {
                        .rg_dis_start = 0,
                        .rg_dis_end = width - 1,
                },
        .yCrop =
                {
                        .rg_dis_start = 0,
                        .rg_dis_end = height - 1,
                },
        .disBufNum = 6,
};

VIN_LDC_ATTR_S LDC_ATTR_SC132GS_BASE = {
        .ldcEnable = 0,
        .ldcPath =
                {
                        .rg_y_only = 0,
                        .rg_uv_mode = 0,
                        .rg_uv_interpo = 0,
                        .reserved1 = 0,
                        .rg_h_blank_cyc = 32,
                        .reserved0 = 0,
                },
        .yStartAddr = 786432,
        .cStartAddr = 1081344,
        .picSize =
                {
                        .pic_w = width - 1,
                        .pic_h = height - 1,
                },
        .lineBuf = 99,
        .xParam =
                {
                        .rg_algo_param_b = 1,
                        .rg_algo_param_a = 1,
                },
        .yParam =
                {
                        .rg_algo_param_b = 1,
                        .rg_algo_param_a = 1,
                },
        .offShift =
                {
                        .rg_center_xoff = 0,
                        .rg_center_yoff = 0,
                },
        .xWoi = {.rg_start = 0, .reserved1 = 0, .rg_length = width - 1, .reserved0 = 0},
        .yWoi = {.rg_start = 0, .reserved1 = 0, .rg_length = height - 1, .reserved0 = 0}
};
