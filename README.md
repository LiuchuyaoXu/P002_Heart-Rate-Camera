# Acceleration-Triggered Camera

Liuchuyao Xu \
Robinson College \
lx242 \

The project was originally called "Heart-Rate-Triggered Camera", where the KL03Z monitors the heart rate of the user (using MAXREFDES117#) and transmits a Bluetooth signal if it changes sharply. The signal then triggers a camera. Due to a broken heart rate sensor, an accelerometer is used instead. \

"firmware" contains a Keil uVision5 project for the firmware of the KL03Z. \
"firmware/RTE" contains libraries used by the firmware. \
"firmware/main.c" contains code written for the application. \
"software" contains the Python code for opening the Bluetooth serial port and taking photos using the webcam. \
