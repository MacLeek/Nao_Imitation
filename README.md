# Nao_Imitation

This is what I've done with my primesense 3d camera sensor and webots simulator this weekend, just for a quick and interesting demo. 
Currently it can only imitate your arm movements.
I'll try to add lower limbs movements imitation and also make the Nao robot move smoothly using filters algorithm when I have time.

## collect_joints.py
This script will read people's skeleton data from tf and transform from kinect's coordinates to Nao's coordinates,then publish the data on /nao

## nao.py
This script will subscribe to the /nao and connect to webots to make the fake nao robot move its arms.
