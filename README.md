# RMDx8Arduino
RMD-X8モータのCAN通信用コマンドのクラス

# Requirement
**Hardware**
* Arduino UNO R3
* Sparkfun CAN-bus shield​
* RMD-X8, RND-X8 pro MOTOR
* 24-48 V

**Software**
* CAN-BUS Shield

# Installation
1. CAN-BUS Shield
   Arduino IDE のライブラリマネージャーからインストール

2. 本リポジトリをクローンもしくはダウンロードし、 \
Windows :` C:\Users\ユーザー名\Documents\Arduino\libraries\` \
Linux : `/snap/arduino/41/Arduino/libraries/`
に保存

# Usage
C:\Users\ユーザー名\Documents\Arduino\librariesのディレクトリに配置されていれば、\
プログラムにて
`#include <RMDx8Arduino.h> `
で読み込むことができる。

# Note
全部のコマンドがあるわけではない\

# Reference
コマンドのチートシート
https://www.dropbox.com/s/2yzt90i10d6dn27/RMD%20servo%20motor%20control%20protocol%20%28CAN%20BUS%20%29V1.61.pdf?dl=0