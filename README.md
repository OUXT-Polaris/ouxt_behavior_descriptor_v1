# ouxt_behavior_descriptor_v1

## Function
- load behavior tree from xml file
    - xml files are generated or edited by Groot
- abstract blackboard description and smart evaluation
    - evaluation by lua

## Completed descriptor example
~~~yaml
behavior:
    description : "$(find-pkg-share robotx_setup)/root.xml"
    blackboard:
        - input : next_goal
          eval: "between(filter(find(red,buoy), local_position.z >=0), filter(find(green,buoy), local_position.z >=0))"    # <- luaで評価    
objects:
    - uuid : 00000000
      attributes:
       - red
       - bouy
      pose:
        position :
           x : 0
           y : 0
           z : 0
    - uuid : 00000001
      attributes:
       - green
       - bouy
~~~

## evaluation API reference(not implemented)
### variables
| name | description |
| --- | --- |
| l_pos | ship position in local coordinate system |
| l_vel | ship velocity in local coordinate system |
| w_pos | ship position in world coordinate system |
| w_vel | ship velocity in world coordinate system |
| theta | ship angle in world coordinate system (radian) |

### functions
| name | description |
| --- | --- |
| between(pos1,pos2) | |
| filter(objects,eval)||
| find(attributes)||
| int_div(pos1,pos2,ratio)| internally dividing point |
| distance(pos1,pos2)||
| angle_diff(angle1,angle2)||
