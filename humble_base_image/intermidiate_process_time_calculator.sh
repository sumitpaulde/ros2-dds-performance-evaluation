#!/usr/bin/bash

echo "ROS2 Performance Analysis Experiment Script has been Started"
printf "\n"
echo "++++++++++++++++++++++++++++++++++++++++++++++++++++"

v1=$1
v2=$2
v3=$3

export RMW

 while :
	do
	echo "Please select a DDS-Implementation to RUN the experiments"
	echo "Select the DDS-Implementation according to the Docker Image."
	echo "1. Eclipse-Cyclone-DDS"
	echo "2. eProsima-FAST-RTPS"
	echo "3. RTI-Connext-DDS"
	echo -n "Type 1 or 2 or 3 :"
	read -n 1 -t 30 a
	printf "\n"
		case $a in
	 		1* )    RMW="rmw_cyclonedds_cpp";;
 
			2* )    RMW="rmw_fastrtps_cpp";;
 
			3* )     RMW="rmw_connextdds";;

 
			* )     echo "Try again.";;
		esac

echo "-----------------------------------"
echo "RMW selected: "$RMW
echo "-----------------------------------"


v1=$s
v2=$b
v3=$i

export filetype

 while :
	do
	echo "Please select a file type to RUN the experiment Publisher"
	echo "s. String data type"
	echo "b. Binary data type"
	echo "i. IMU data type"
	echo -n "Type s or b or i :"
	read -n 1 -t 30 a
	printf "\n"
		case $a in
	 		s* )    filetype="string";;
 
			b* )    filetype="binary";;
 
			i* )     filetype="imu";;

 
			* )     echo "Try again.";;
		esac

echo "-----------------------------------"
echo "Filetype selected: "$filetype
echo "-----------------------------------"


echo "ROS2 Transfer rate Analysis Publisher Started"
echo "++++++++++++++++++++++++++++++++++++++++++++++++++++"
printf "Please check the corresponding bridiging service has been started \n if not please start and then press 'y' to start the Experiment"
printf "\n"
echo "++++++++++++++++++++++++++++++++++++++++++++++++++++"


	while : ; do
		read -n 1 k <&1
		if [[ $k = y ]] ; then
			printf "\nStarting New Experiement\n"
		break
		fi
	done


binary="binary"
string="string"
imu="imu"

if [ $filetype = $binary ];
			
	then

echo "++++++++++++++++++++++++++++++++++++++++++++++++++++"
echo "+ Experiment Running for Binary Files +"
echo "++++++++++++++++++++++++++++++++++++++++++++++++++++"

		for domain in "same_domain" "different_domain"
			do
			domain_id=10
	
			for imagesize in 33kb 67kb 87kb 145kb 294kb 502kb 1mb
			do

				for rate in 1 2 4 8 10 12 15 20 100 1000
				do 
				RMW_IMPLEMENTATION=$RMW ROS_DOMAIN_ID=$domain_id ros2 run cam_stream intermidiate_processing_time_collector_binary --ros-args -p imagesize:=$imagesize -p hz:=$rate -p domain:=$domain

			#
			printf "\n"
			printf "\n"
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
			echo "Experiment done for "$imagesize" and rate "$rate"hz for domain: "$domain" with domain id: "$domain_id
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
			done
		done
		
	done
fi	

if [ $filetype = $string ];

	then

	echo "++++++++++++++++++++++++++++++++++++++++++++++++++++"
echo "+ Experiment Running for String Type +"
echo "++++++++++++++++++++++++++++++++++++++++++++++++++++"


	for domain in "same_domain" "different_domain"
		do
		domain_id=10


		for rate in 10 20 40 80 100 200 500 1000
			do 
			RMW_IMPLEMENTATION=$RMW ROS_DOMAIN_ID=$domain_id ros2 run cam_stream intermidiate_processing_time_collector_string  --ros-args -p hz:=$rate -p domain:=$domain

			echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
			echo "Experiment completed for String Type- "$rate"hz for domain: "$domain" with domain id: "$domain_id
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
		done
		
		for rate in 10 20 40 80 100 200 500 1000
			do 
			RMW_IMPLEMENTATION=$RMW ROS_DOMAIN_ID=$domain_id ros2 run cam_stream intermidiate_processing_time_collector_string  --ros-args  -p hz:=$rate -p domain:=$domain
		
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
			echo "Experiment completed for String Type- "$rate"hz for domain: "$domain" with domain id: "$domain_id
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
		done
		
		for rate in 10 20 40 80 100 200 500 1000
			do RMW_IMPLEMENTATION=$RMW ROS_DOMAIN_ID=$domain_id ros2 run cam_stream intermidiate_processing_time_collector_string  --ros-args -p hz:=$rate -p domain:=$domain
			
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
			echo "Experiment completed for String Type- "$rate"hz for domain: "$domain" with domain id: "$domain_id
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
		done
		
		for rate in 10 20 40 80 100 200 500 1000
			do RMW_IMPLEMENTATION=$RMW ROS_DOMAIN_ID=$domain_id ros2 run cam_stream intermidiate_processing_time_collector_string  --ros-args -p hz:=$rate -p domain:=$domain
			
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
			echo "Experiment completed for String Type- "$rate"hz for domain: "$domain" with domain id: "$domain_id
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
		done
	done
fi

if [ $filetype = $imu ];


then
echo "++++++++++++++++++++++++++++++++++++++++++++++++++++"
echo "+ Experiment Running for IMU Files +"
echo "++++++++++++++++++++++++++++++++++++++++++++++++++++"


	for domain in "same_domain" "different_domain"
		do		
			domain_id=10

			for rate in 10 20 40 80 100 200 500 1000
			do RMW_IMPLEMENTATION=$RMW ROS_DOMAIN_ID=$domain_id ros2 run cam_stream intermidiate_processing_time_collector_imu --ros-args -p hz:=$rate -p domain:=$domain
			
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
			echo "Experiment completed for String Type- "$rate"hz for domain: "$domain" with domain id: "$domain_id
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
			done
	done


fi 

	


 
exit
 
	
 	done
 done







































#/home/spaul/images/images_2mb /home/spaul/images/images_33kb /home/spaul/images/images_1mb /home/spaul/images/images_4mb /home/spaul/images/images_2mb /home/spaul/images/images_67kb /home/spaul/images/images_87kb /home/spaul/images/images_145kb /home/spaul/images/images_294kb /home/spaul/images/images_502kb
