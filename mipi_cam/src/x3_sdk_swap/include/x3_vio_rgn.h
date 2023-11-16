/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_VIO_RGN_H_
#define X3_VIO_RGN_H_


int x3_rgn_init(RGN_HANDLE Handle, RGN_CHN_S *chn, RGN_ATTR_S *pstRegion, RGN_CHN_ATTR_S *pstChnAttr);
int x3_rgn_uninit(RGN_HANDLE Handle, RGN_CHN_S *chn);

#endif // X3_VIO_RGN_H_
