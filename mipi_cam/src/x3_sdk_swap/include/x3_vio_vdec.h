/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_VIO_VDEC_H_
#define X3_VIO_VDEC_H_

#ifdef SW_VDEC
#include "vio/hb_comm_vdec.h"
#include "vio/hb_vdec.h"

#include "x3_sdk_wrap.h"

int x3_vdec_init(VDEC_CHN vdecChn, VDEC_CHN_ATTR_S* vdecChnAttr);
int x3_vdec_start(VDEC_CHN vdecChn);
int x3_vdec_stop(VDEC_CHN vdecChn);
int x3_vdec_deinit(VDEC_CHN vdecChn);

#endif
#endif // X3_VIO_VDEC_H_

