# XsensPlugin

The `XsensPlugin` handles integration between `Xsens` sensors and `mc_rtc`.

This package depends on:
- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)
- [xsens_streaming](https://github.com/arntanguy/xsens_streaming)

## Installation

After installing the afformentioned dependencies,

```sh
git clone https://github.com/arntanguy/mc_xsens_plugin.git
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j8
sudo make install
```

## Running instructions

Run using your `mc_rtc` interface of choice, add `XsensPlugin` to the Plugins configuration entry or enable the autoload option
