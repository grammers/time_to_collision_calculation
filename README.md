# time_to_collision_calculatin
A method to callculate relative time to collision. based on bonding boxes trecking

It is ment to be pared with [bebop atonumus](https://bebop-autonomy.readthedocs.io/en/latest/)

## requierment
For it to work do you need to download thes [files](https://drive.google.com/drive/folders/1Z-A-dWUQ_Z43smqZgl2QOppFCqUcJCl3?usp=sharing) and change the variabel `PATH` in yolo_detecter.py

## Usgae
To run start all nodes.

`bebop_usgaes_api` is to start and stop the bebop.
Comands are `takeoff`, `land` and `stop`.

`turet` is a controll loop that is hovering and changes heading to avoid poinitin att pedestrians but yout keaping a goal heading in minde.

`heading_prediction` sugest a heading to the controller.

`collision_calculation` calcuats the aprotching objects heading and ther change in area.

`tracker` is tracking the bounding boxes from `yolo`

`yolo` a yolo implimentation in opencv that pruduses bounding boxes.

`bb_traing` is not in use, it is an old implementation form bounding boxes and tracking.

## modularity
The system is desigen to be modular. Every module can be chanded to a better one or some thig to test ass long ass input and output is constant.
To imporv preformes should you look in to improving the detection and tracking.
