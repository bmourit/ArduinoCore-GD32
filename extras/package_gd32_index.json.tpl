{
  "packages": [
    {
      "email": "",
      "help": {
        "online": "https://github.com/bmourit/ArduinoCore-GD32"
      },
      "maintainer": "Some random folks",
      "name": "GD32Community",
      "platforms": [
        {
          "architecture": "gd32",
          "boards": [
            {
              "name": "GD32F303ZE EVAL"
            },
            {
              "name": "GD32F307VG MBED"
            },
            {
              "name": "GD32F303CC Generic"
            },
	    {
	      "name": "GD32F303RE Generic"
	    }
          ],
          "category": "Contributed",
          "help": {
            "online": ""
          },
          "name": "GigaDevice GD32",
          "toolsDependencies": [
            {
              "packager": "GD32Community",
              "name": "xpack-arm-none-eabi-gcc",
              "version": "13.2.1-1.1"
            },
            {
              "packager": "GD32Community",
              "name": "xpack-openocd",
              "version": "0.12.0-2"
            },
            {
              "packager": "GD32Community",
              "version": "0.10.0-arduino1",
              "name": "dfu-util"
            }
          ],
          "url": "$url",
          "archiveFilename": "$archiveFilename",
          "size": "$size",
          "checksum": "$checksum",
          "version": "0.0.1"
        }
      ],
      "tools": [
        {
          "name": "xpack-arm-none-eabi-gcc",
          "version": "13.2.1-1.1",
          "systems": [
            {
              "host": "arm-linux-gnueabihf",
              "url": "https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v13.2.1-1.1/xpack-arm-none-eabi-gcc-13.2.1-1.1-linux-arm.tar.gz",
              "archiveFileName": "xpack-arm-none-eabi-gcc-13.2.1-1.1-linux-arm.tar.gz",
              "checksum": "SHA-256:9a6db147c34f7ea668cc37a139d2667a58f8b2bbee2359f23e48ffd300f8fc2f",
              "size": "257683778"
            },
            {
              "host": "aarch64-linux-gnu",
              "url": "https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v13.2.1-1.1/xpack-arm-none-eabi-gcc-13.2.1-1.1-linux-arm64.tar.gz",
              "archiveFileName": "xpack-arm-none-eabi-gcc-13.2.1-1.1-linux-arm64.tar.gz",
              "checksum": "SHA-256:ab7f75d95ead0b1efb7432e7f034f9575cc3d23dc1b03d41af1ec253486d19de",
              "size": "265190134"
            },
            {
              "host": "x86_64-mingw32",
              "url": "https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v13.2.1-1.1/xpack-arm-none-eabi-gcc-13.2.1-1.1-win32-x64.zip",
              "archiveFileName": "xpack-arm-none-eabi-gcc-13.2.1-1.1-win32-x64.zip",
              "checksum": "SHA-256:56b18ccb0a50f536332ec5de57799342ff0cd005ca2c54288c74759b51929e4f",
              "size": "306340878"
            },
            {
              "host": "aarch64-apple-darwin",
              "url": "https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v13.2.1-1.1/xpack-arm-none-eabi-gcc-13.2.1-1.1-darwin-arm64.tar.gz",
              "archiveFileName": "xpack-arm-none-eabi-gcc-13.2.1-1.1-darwin-arm64.tar.gz",
              "checksum": "SHA-256:d4ce0de062420daab140161086ba017642365977e148d20f55a8807b1eacd703",
              "size": "258277148"
            },
            {
              "host": "x86_64-apple-darwin",
              "url": "https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v13.2.1-1.1/xpack-arm-none-eabi-gcc-13.2.1-1.1-darwin-x64.tar.gz",
              "archiveFileName": "xpack-arm-none-eabi-gcc-13.2.1-1.1-darwin-x64.tar.gz",
              "checksum": "SHA-256:1ecc0fd6c31020aff702204f51459b4b00ff0d12b9cd95e832399881d819aa57",
              "size": "262217286"
            },
            {
              "host": "x86_64-pc-linux-gnu",
              "url": "https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v13.2.1-1.1/xpack-arm-none-eabi-gcc-13.2.1-1.1-linux-x64.tar.gz",
              "archiveFileName": "xpack-arm-none-eabi-gcc-13.2.1-1.1-linux-x64.tar.gz",
              "checksum": "SHA-256:1252a8cafe9237de27a765376697230368eec21db44dc3f1edeb8d838dabd530",
              "size": "268158135"
            },
          ]
        },
        {
          "name": "xpack-openocd",
          "version": "0.12.0-2",
          "systems": [
            {
              "host": "arm-linux-gnueabihf",
              "url": "https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v0.12.0-2/xpack-openocd-0.12.0-2-linux-arm.tar.gz",
              "archiveFileName": "xpack-openocd-0.12.0-2-linux-arm.tar.gz",
              "checksum": "SHA-256:bc123785e9009caa6ce166afb4298104a19c20f734ae044f3caf37896d401a45",
              "size": "2451490"
            },
            {
              "host": "aarch64-linux-gnu",
              "url": "https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v0.12.0-2/xpack-openocd-0.12.0-2-linux-arm64.tar.gz",
              "archiveFileName": "xpack-openocd-0.12.0-2-linux-arm64.tar.gz",
              "checksum": "SHA-256:d4fad1505ef299f511eb2324c6e2c7bbef1101a22155b70478d89428c3fb515b",
              "size": "2529589"
            },
            {
              "host": "x86_64-mingw32",
              "url": "https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v0.12.0-2/xpack-openocd-0.12.0-2-win32-x64.zip",
              "archiveFileName": "xpack-openocd-0.12.0-2-win32-x64.zip",
              "checksum": "SHA-256:6cb8a5f9290c59be3e42a6f1066f23ff5a9dbe5b83bdecd0e9d4c7dcc1233517",
              "size": "2990739"
            },
            {
              "host": "aarch64-apple-darwin",
              "url": "https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v0.12.0-2/xpack-openocd-0.12.0-2-darwin-arm64.tar.gz",
              "archiveFileName": "xpack-openocd-0.12.0-2-darwin-arm64.tar.gz",
              "checksum": "SHA-256:7b35c4a01dc8e74fbf3a217f78bd01d61112a09be2f2e2ba9e8e737b12c84562",
              "size": "2213119"
            },
            {
              "host": "x86_64-apple-darwin",
              "url": "https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v0.12.0-2/xpack-openocd-0.12.0-2-darwin-x64.tar.gz",
              "archiveFileName": "xpack-openocd-0.12.0-2-darwin-x64.tar.gz",
              "checksum": "SHA-256:71942e8db55ed68cad45e07c2065e6c7cc0e127797ee26c298ec4dcaebc9232c",
              "size": "2289633"
            },
            {
              "host": "x86_64-pc-linux-gnu",
              "url": "https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v0.12.0-2/xpack-openocd-0.12.0-2-linux-x64.tar.gz",
              "archiveFileName": "xpack-openocd-0.12.0-2-linux-x64.tar.gz",
              "checksum": "SHA-256:1dc8e63694204b73107fe5b23267238af2c6ce14726e4aa244a2123a98805335",
              "size": "2581203"
            },
          ]
        },
	        {
          "name": "dfu-util",
          "version": "0.10.0-arduino1",
          "systems": [
            {
              "host": "i386-apple-darwin11",
              "url": "http://downloads.arduino.cc/tools/dfu-util-0.10.0-arduino1-osx.tar.bz2",
              "archiveFileName": "dfu-util-0.10.0-arduino1-osx.tar.bz2",
              "size": "73921",
              "checksum": "SHA-256:7562d128036759605828d64b8d672d42445a8d95555c4b9ba339f73a1711a640"
            },
            {
              "host": "x86_64-apple-darwin11",
              "url": "http://downloads.arduino.cc/tools/dfu-util-0.10.0-arduino1-osx.tar.bz2",
              "archiveFileName": "dfu-util-0.10.0-arduino1-osx.tar.bz2",
              "size": "73921",
              "checksum": "SHA-256:7562d128036759605828d64b8d672d42445a8d95555c4b9ba339f73a1711a640"
            },
            {
              "host": "arm-linux-gnueabihf",
              "url": "http://downloads.arduino.cc/tools/dfu-util-0.10.0-arduino1-arm.tar.bz2",
              "archiveFileName": "dfu-util-0.10.0-arduino1-arm.tar.bz2",
              "size": "272153",
              "checksum": "SHA-256:f1e550f40c235356b7fde1c59447bfbab28f768915d3c14bd858fe0576bfc5a9"
            },
            {
              "host": "aarch64-linux-gnu",
              "url": "http://downloads.arduino.cc/tools/dfu-util-0.10.0-arduino1-arm64.tar.bz2",
              "archiveFileName": "dfu-util-0.10.0-arduino1-arm64.tar.bz2",
              "size": "277886",
              "checksum": "SHA-256:ebfbd21d3030c500da1f83b9aae5b8c597bee04c3bde1ce0a51b41abeafc9614"
            },
            {
              "host": "x86_64-linux-gnu",
              "url": "http://downloads.arduino.cc/tools/dfu-util-0.10.0-arduino1-linux64.tar.bz2",
              "archiveFileName": "dfu-util-0.10.0-arduino1-linux64.tar.bz2",
              "size": "77184",
              "checksum": "SHA-256:13ef2ec591c1e8b0b7eb0a05da972ecd6695016e7a9607e332c7553899af9b4a"
            },
            {
              "host": "i686-linux-gnu",
              "url": "http://downloads.arduino.cc/tools/dfu-util-0.10.0-arduino1-linux32.tar.bz2",
              "archiveFileName": "dfu-util-0.10.0-arduino1-linux32.tar.bz2",
              "size": "81826",
              "checksum": "SHA-256:43599ec60c000e9ef016970a496d6ab2cbbe5a8b7df9d06ef3114ecf83f9d123"
            },
            {
              "host": "i686-mingw32",
              "url": "http://downloads.arduino.cc/tools/dfu-util-0.10.0-arduino1-windows.tar.bz2",
              "archiveFileName": "dfu-util-0.10.0-arduino1-windows.tar.bz2",
              "size": "464314",
              "checksum": "SHA-256:90816b669273ae796d734a2459c46bb340d4790783fd7aa01eb40c0443f1a9b1"
            }
          ]
        }
      ],
      "websiteURL": "https://github.com/bmourit/ArduinoCore-GD32/"
    }
  ]
}
