/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Copyright (C) 2007-2008 by T.TAKASU, All rights reserved.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_LAMBDA_H
#define LIBSWIFTNAV_LAMBDA_H

#include <swiftnav/common.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

int lambda_reduction(int n, const double *Q, double *Z);
int lambda_solution(int n, int m, const double *a, const double *Q, double *F,
                    double *s);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* LIBSWIFTNAV_LAMBDA_H */
