# comps
Fork of the Constrained Manipulation Planning Suite (CoMPS) by Dmitry Berenson

# Dependency

NEWMAT:

`sudo apt-get install libnewmat10-dev`

cdd lib:
https://www.inf.ethz.ch/personal/fukudak/cdd_home/
`
sudo apt-get install libgmp3-dev
wget ftp://ftp.math.ethz.ch/users/fukudak/cdd/cddlib-094h.tar.gz
tar -xvf cddlib-094h.tar.gz
cd cddlib-094h
./configure
make -j4
sudo make install
cd /usr/local/include
sudo mkdir cdd
sudo mv cdd_f.h cddmp_f.h cddtypes_f.h cdd.h cddmp.h cddtypes.h setoper.h cdd
`
