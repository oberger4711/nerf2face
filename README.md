## Install dlib on Raspberry Pi 4
dlib face detections produces less false positives than the OpenCV detector.
Unfortunately, dlib face detection runs very slowly on Raspberry Pi 4.
One reason for this is that AVX instruction set is not supported and the available NEON instructions are not implemented in dlib.
I hacked in NEON instructions (float32x4 vectors) for spatial filtering.
Speedup is approximately 2: From 6 s to 3 s.
This is still way too slow.
Maybe more is possible with the NEON-VFPv4 instructions (float16x8) and running fewer detectors but for now I stick to the OpenCV face detector based on Haar cascades.

Enable NEON SIMD instructions like this.
```
pi@raspberrypi:~/dlib-19.18/build $ cmake -DUSE_NEON_INSTRUCTIONS=ON -DDLIB_NO_GUI_SUPPORT=ON ..
pi@raspberrypi:~/dlib-19.18/build $ cmake --build . --config Release
pi@raspberrypi:~/dlib-19.18/build $ sudo make install
```
