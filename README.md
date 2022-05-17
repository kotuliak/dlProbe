LTE DL SNIFFER
======

Implementation of LTE downlink sniffer based on srsRAN library. srsRAN is a 4G/5G software radio suite developed by [SRS](http://www.srs.io).

For license details, see LICENSE file.

Build
=====

```
mkdir build
cd build
cmake ../
make
sudo ldconfig
sudo make install
```

Run
===
```
sudo dlProbe dlProbe.conf
```
You can find example config file [here](dlProbe/dlProbe.conf.example).

dlProbe was tested on a operating base station with USRP B210 connected to a PC with i9-10900KF processor.

Support
=======

Contact me at: martin.kotuliak (at) inf.ethz.ch

Citing
======

```
@inproceedings{kotuliak2022ltrack,
  title={LTrack: Stealthy Tracking of Mobile Phones in LTE},
  author={Kotuliak, Martin and Erni, Simon and Leu, Patrick and Roeschlin, Marc and Capkun, Srdjan},
  booktitle={31st USENIX Security Symposium (USENIX 2022)},
  year={2022}
}
```

