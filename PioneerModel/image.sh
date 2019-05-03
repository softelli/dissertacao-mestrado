
#!/bin/bash
for i in {0..4};
do
        rosrun image_view image_view image:=/p$i/camera/rgb/image_raw &
done
wait
echo "all proccess complete"
 
