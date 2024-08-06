/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PRINTING_UTILS_H
#define LIBSWIFTNAV_PRINTING_UTILS_H

#include <swiftnav/common.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void print_double_mtx(double *m, u32 _r, u32 _c);
void print_pearson_mtx(double *m, u32 dim);
void print_s32_mtx_diff(u32 m, u32 n, s32 *mat1, s32 *mat2);
void print_s32_mtx(s32 *mat, u32 m, u32 n);
void print_s32_gemv(u32 m, u32 n, s32 *M, s32 *v);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* LIBSWIFTNAV_PRINTING_UTILS_H */
