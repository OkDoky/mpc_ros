#!/usr/bin/bash

cd /root/
wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.14.4.tar.gz
tar -xvzf Ipopt-3.14.4.tar.gz 
cd Ipopt-releases-3.14.4/
apt install cppad gfortran
git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git
cd ThirdParty-Mumps/
./get.Mumps 
./configure
make -j4 && make test && make install

cd ..
./configure --prefix=/usr/local --with-mumps-lib="-L/usr/local/lib -ldmumps -lmumps_common -lpord -lmpiseq -lesmumps -lcamseq" --with-mumps-incdir="/usr/local/include" ADD_CXXFLAGS="-I/usr/local/include"
make -j4
make test
make install

## cause cppad use coin/IpIpoptAppication.hpp
mv /usr/local/include/coin-or /usr/local/include/coin

echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib'>> ~/.bashrc
soruce ~/.bashrc
