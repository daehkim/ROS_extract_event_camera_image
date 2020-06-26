<How to use>

We should run the "roscore" and "dvs_renderer" in separate terminals before 
export the images.
>roscore
>rosrun dvs_renderer dvs_renderer events:=/cam0/events

After that, we can run the python script with this command
>python extract_event_img.py



------------------------------------------------------------------------------
<Explanation of the export.launch>

In the export.launch file, 2nd line run the play subcommand in the rosbag 
package. It runs all the topics inside the /tmp/out.bag file.

The 3rd line of export.launch means it will run the extract_images subcommand 
in the image_view package. The node name will be assigned as "extract".

The 4th line of export.launch is important. In the 2nd line of export.launch, 
out.bag file will be read and send the data for the each topic. The EventArray
will be sent to the "/cam0/events" directory. That's why we set the 
dvs_renderer's input event directory as "/cam0/events". 
(Refer the 6th line of this document)
As a result of the dvs_renderer, the event picture will go to the 
/dvs_rendering/. That's why we set the remap as /dvs_rendering/ directory.

The 5th line is about the event image's frame setting. We generate the image
with 60 fps, so set the sec_per_frame as 1/60=0.016

As a summary, EventArray information extracted from the out.bag file and
be delivered to the /cam0/events (which is also the topic's name). The 
dvs_renderer read the data from the /cam0/events and generate the image, and 
send it to the /dvs_rendering/ directory. Finally, image_view read the image
from the /dvs_rendering/ and generate the png files to the ~/.ros/ directory
with the name of frame~~.
