#!/usr/bin/env python3
from voice_module import recognize_from_wav
import os

test_dir = "/home/robotics/sagittarius_ws/src/final_project/test_data/colour"

for fname in sorted(os.listdir(test_dir)):
    if fname.endswith(".wav"):
        wav_path = os.path.join(test_dir, fname)
        recognize_from_wav(wav_path)

