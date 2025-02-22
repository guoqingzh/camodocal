
#selected_dataset=lifelong-robotics
#selected_data=cafe1-2_two_fisheyes

selected_dataset=vslam-recordings
selected_data=SL03
num_camera=2
cam_index=0

if [[ $1 -eq "" ]]
then
    echo "Missing path to input data !"
    exit
fi
echo "************************************************"
echo "Input data ${selected_data}, sanity checking ..."
echo "************************************************"
python3 checker.py ${num_camera} $1/data/${selected_dataset}/${selected_data}/camodocal_input_data/
echo "Press any key to continue ..."
read -n 1

extrinsic_calib -c $1/data/${selected_dataset}/${selected_data}/camodocal_input_caminfo --camera-count ${num_camera} --camera-index ${cam_index} -m 30 --input $1/data/${selected_dataset}/${selected_data}/camodocal_input_data/ --keydist 0.1 --begin-stage 0  --start-ts  -v 


#extrinsic_calib -c $1/data/${selected_data}/camodocal_input_caminfo -m 50 --input $1/data/${selected_data}/camodocal_input_data/ --keydist 0.4 -v
