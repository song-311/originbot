# Copyright 2024 GuYueHome
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pure, ROS-independent utility functions for the traffic-light detector."""

import numpy as np


def clamp_roi(x, y, w, h, img_w, img_h):
    """Clamp an ROI so that it lies fully within the image bounds."""
    x = max(0, min(int(x), img_w - 1))
    y = max(0, min(int(y), img_h - 1))
    w = max(1, min(int(w), img_w - x))
    h = max(1, min(int(h), img_h - y))
    return x, y, w, h


def mask_ratio(mask):
    """Return the fraction of non-zero pixels in *mask*."""
    return float(np.count_nonzero(mask)) / float(mask.size)


def order_corners(pts):
    """Return four corner points in (top-left, top-right, bottom-right, bottom-left) order.

    Sorting strategy: split the four points into the two with smaller y (top pair)
    and the two with larger y (bottom pair), then order each pair left-to-right.
    This is robust to perspective skew and rotated cards.
    """
    pts = pts.reshape(4, 2).astype(np.float32)
    # Sort by y coordinate; top two points have smaller y
    sorted_by_y = pts[np.argsort(pts[:, 1])]
    top = sorted_by_y[:2]
    bottom = sorted_by_y[2:]
    # Within each pair, sort left-to-right by x
    tl, tr = top[np.argsort(top[:, 0])]
    bl, br = bottom[np.argsort(bottom[:, 0])]
    return np.array([tl, tr, br, bl], dtype=np.float32)
