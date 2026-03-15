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

"""Unit tests for pure utility functions in traffic_light_detector_node.

These tests do not require a running ROS 2 environment and exercise the
helper functions that underpin the detection logic, as well as verifying
that the always-publish (UNKNOWN) semantics are preserved.
"""

import numpy as np
import pytest

from originbot_traffic_light._utils import clamp_roi, mask_ratio, order_corners


class TestClampRoi:
    """Tests for the clamp_roi helper."""

    def test_no_clamp_needed(self):
        x, y, w, h = clamp_roi(10, 20, 100, 80, 640, 480)
        assert (x, y, w, h) == (10, 20, 100, 80)

    def test_clamp_negative_x(self):
        x, y, w, h = clamp_roi(-5, 0, 50, 50, 640, 480)
        assert x == 0

    def test_clamp_negative_y(self):
        x, y, w, h = clamp_roi(0, -10, 50, 50, 640, 480)
        assert y == 0

    def test_clamp_width_beyond_right_edge(self):
        x, y, w, h = clamp_roi(600, 0, 200, 50, 640, 480)
        assert x + w <= 640

    def test_clamp_height_beyond_bottom_edge(self):
        x, y, w, h = clamp_roi(0, 450, 50, 200, 640, 480)
        assert y + h <= 480

    def test_minimum_width_one(self):
        _, _, w, _ = clamp_roi(0, 0, 0, 50, 640, 480)
        assert w >= 1

    def test_minimum_height_one(self):
        _, _, _, h = clamp_roi(0, 0, 50, 0, 640, 480)
        assert h >= 1


class TestMaskRatio:
    """Tests for the mask_ratio helper."""

    def test_all_zero_mask_returns_zero(self):
        mask = np.zeros((100, 100), dtype=np.uint8)
        assert mask_ratio(mask) == pytest.approx(0.0)

    def test_all_nonzero_mask_returns_one(self):
        mask = np.ones((100, 100), dtype=np.uint8) * 255
        assert mask_ratio(mask) == pytest.approx(1.0)

    def test_half_nonzero_returns_half(self):
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[:50, :] = 255
        assert mask_ratio(mask) == pytest.approx(0.5)


class TestOrderCorners:
    """Tests for the order_corners helper."""

    def test_axis_aligned_rectangle(self):
        pts = np.array([[0, 0], [100, 0], [100, 50], [0, 50]], dtype=np.float32)
        ordered = order_corners(pts)
        tl, tr, br, bl = ordered
        assert tl[0] < tr[0], 'top-left x should be left of top-right x'
        assert bl[0] < br[0], 'bottom-left x should be left of bottom-right x'
        assert tl[1] < bl[1], 'top-left y should be above bottom-left y'

    def test_output_shape(self):
        pts = np.array([[0, 0], [10, 0], [10, 5], [0, 5]], dtype=np.float32)
        ordered = order_corners(pts)
        assert ordered.shape == (4, 2)

    def test_shuffled_input_gives_same_result(self):
        pts = np.array([[0, 0], [100, 0], [100, 50], [0, 50]], dtype=np.float32)
        shuffled = pts[[2, 0, 3, 1]]
        np.testing.assert_array_almost_equal(
            order_corners(pts), order_corners(shuffled)
        )
