#!/bin/bash

avg=0
i=0
max=100

while [ $i -lt $max ]
do
    rv=$(hcitool rssi C8:3C:85:A7:6A:D7)
    num=$(echo $rv | grep -o -E '[+-]?[0-9]+')
    avg=$(echo "$avg + $num" | bc)
    i=$(echo "$i + 1" | bc)
done

avg=$(echo "$avg / 100" | bc)

echo "$avg" > test.txt
