#!/bin/sh

SCRIPTPATH=$(pwd)
echo $SCRIPTPATH

cd $SCRIPTPATH/str/
bins=(*.bin)                 
cd ../

for bin in "${bins[@]}"; do
  file=$(echo "${bin}" | cut -d "." -f 1)
  cd info
  mkdir $file
  cd ../

  python decodeAndEstimateRate.py $SCRIPTPATH $SCRIPTPATH/str/$bin $SCRIPTPATH/info/$file
  echo "$file done"
done

python selectParameters.py $SCRIPTPATH/info