#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2025  Sumanth Nagulavancha, Ignacio Vizzo, Tiziano Guadanino, Saurabh Gupta
# Permission is hereby granted, free of charge, to any person
# obtaining a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including without
# limitation the rights to use, copy, modify, merge, publish, distribute,
# sublicense, and/or sell copies of the Software, and to permit persons to whom
# the Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
from pathlib import Path
from typing import Optional

import argh
from datasets.tum import TUMDataset
from downloader import load_vocabulary

from orb_slam3 import ORB_SLAM3


def main(
    data_source: str,
    config: Optional[str] = None,
    vocabulary: Optional[str] = None,
    visualize: bool = False,
):
    if vocabulary is None:
        vocabulary = load_vocabulary()
    if config is None:
        config = str(Path(__file__).parent / "config/TUM1.yaml")

    dataset = TUMDataset(data_source)
    slam = ORB_SLAM3(vocabulary, config, "rgbd", visualize)  # initialize
    print(f"done intializing")
    for value in dataset:
        rgb, depth, current_timestamp = value
        slam.TrackRGBD(image=rgb, depthmap=depth, timestamp=current_timestamp)

    slam.SaveTrajectoryTUM("tum_trajectory.txt")
    slam.Shutdown()


if __name__ == "__main__":
    argh.dispatch_command(main)
