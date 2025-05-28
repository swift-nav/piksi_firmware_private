/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_ARRAY_BASE_ADDONS_H
#define LIBSWIFTNAV_PVT_ENGINE_ARRAY_BASE_ADDONS_H

void append(const Scalar &element) {
  this->conservativeResize(this->size() + 1);
  this->tail(1) = element;
}

void push_back(const Scalar &element) { this->append(element); }

bool contains(const Scalar &element) const {
  for (const auto &item : *this) {
    if (item == element) {
      return true;
    }
  }
  return false;
}

void clear() { this->conservativeResize(0); }

Scalar *begin() { return this->data(); }

Scalar *end() { return this->data() + this->size(); }

const Scalar *begin() const { return this->data(); }

const Scalar *end() const { return this->data() + this->size(); }

void replace(const Scalar &from, const Scalar &to) {
  for (auto &item : *this) {
    if (item == from) {
      item = to;
    }
  }
}

#endif  // LIBSWIFTNAV_PVT_ENGINE_ARRAY_BASE_ADDONS_H
