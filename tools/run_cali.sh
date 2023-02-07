
selected_data=cafe1-1

extrinsic_calib -c $1/data/${selected_data}/camodocal_input_caminfo --camera-count 2 -m 50 --input $1/data/${selected_data}/camodocal_input_data/ --keydist 0.1
