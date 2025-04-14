#Script for starting all 4 cameras
#put this in the build folder

#If two arugments are passed, assume first IP and second port
#If one argument is passed, assume that it's IP and use standard port 2020
#If no argument is passed, start with standard ip and port

trap 'kill %1; kill %2; kill %3; kill %4' SIGINT
if [ $# -eq 2 ]
then
  ./GulliView -d 0 -n -f Tag16h5 -W 800 -H 448 -V $1 -N $2 &
  ./GulliView -d 1 -n -f Tag16h5 -W 800 -H 448 -V $1 -N $2 &
  ./GulliView -d 2 -n -f Tag16h5 -W 800 -H 448 -V $1 -N $2 &
  ./GulliView -d 3 -n -f Tag16h5 -W 800 -H 448 -V $1 -N $2 &
elif [ $# -eq 1  ]
then
  ./GulliView -d 0 -n -f Tag16h5 -W 800 -H 448 -V $1 &
  ./GulliView -d 1 -n -f Tag16h5 -W 800 -H 448 -V $1 &
  ./GulliView -d 2 -n -f Tag16h5 -W 800 -H 448 -V $1 &
  ./GulliView -d 3 -n -f Tag16h5 -W 800 -H 448 -V $1 &
else
  ./GulliView -d 4 -n -f Tag16h5 -W 800 -H 448 -V 192.168.5.21 -N 2020 &
  ./GulliView -d 1 -n -f Tag16h5 -W 800 -H 448 -V 192.168.5.21 -N 2020 &
  ./GulliView -d 2 -n -f Tag16h5 -W 800 -H 448 -V 192.168.5.21 -N 2020 &
  ./GulliView -d 3 -n -f Tag16h5 -W 800 -H 448 -V 192.168.5.21 -N 2020 &
fi

wait
