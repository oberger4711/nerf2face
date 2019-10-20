## Install DLib on Raspberry Pi 4
Unfortunately, dlib face detection runs very slowly on Raspberry Pi 4.
Enable NEON SIMD instructions like this:
```
pi@raspberrypi:~/dlib-19.18/build $ cmake -DUSE_NEON_INSTRUCTIONS=ON -DDLIB_NO_GUI_SUPPORT=ON ..
pi@raspberrypi:~/dlib-19.18/build $ cmake --build . --config Release
pi@raspberrypi:~/dlib-19.18/build $ sudo make install
```
This does not significantly speed up the algortithm though...
