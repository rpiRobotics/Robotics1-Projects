# Group 26 - Morality
Team Members: Eric Ward

# Running the code

Okay so if you *really* want to run this code, it's a trip. First just ignore anything related to RPI's duckiebot repository, I didn't use that.

So, on your favorite duckiebot,
Clone the current mit duckietown Software repo:

    $ git clone https://github.com/duckietown/Software.git

Follow *their* setup process: https://docs.duckietown.org/DT18/opmanual_duckiebot/out/laptop_setup.html

Once you do that and have the catkin workspace set up, `cd` into `catkin_ws/src` and do 

    $ catkin_create_pkg morality

Delete the folder it generates because you'll paste my code instead.

    $ rm -rf morality

Paste this folder into `catkin_ws/src`. Now `cd` into `catkin_ws` and run `catkin_make` a few times. The parallel build means that it messes up the dependencies and needs multiple passes at building.

Now you can do

    $ roslaunch morality car_control.launch
    
And it might work but it probably won't. It'll probably take some finnicking and know-how on your end to actually get this to run for you.

My suggestion is to setup the duckiebot with the docker workflow, that way you can install their base image and our software repos will be starting from the same point.
Then you can try adding the morality package.

If you want, you can just run the simple path_plan node in isolation more or less alongside the matlab script `test_path_plan.m`.
This only requires you did the build locally. Then after you `catkin_make`d enough, run rosinit in matlab or roscore somewhere on the local machine. Then

    $ rosrun morality path_plan
 
Now you can run the `test_path_plan.m` script in matlab. Mess with the gain values using rosparam.

    $ rosparam list
    
will show you the names, then use `rosparam set ...` to set the floats to what you want.
That's all from me.

Thanks for a semester.
