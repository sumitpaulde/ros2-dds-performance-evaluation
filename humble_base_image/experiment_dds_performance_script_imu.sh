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

	
printf "\n"
echo "###############################################################"
echo "Please Start the IMU Subscriber"
echo "###############################################################"
printf "Please check the corresponding bridiging service has been started \n if not please start and then press 'y' to start the Experiment"	

	while : ; do
	read -n 1 k <&1
	if [[ $k = y ]] ; then
	  printf "\nStarting New Experiement\n"
	break
	fi
	done

echo "++++++++++++++++++++++++++++++++++++++++++++++++++++"
echo "Experiment Starting for IMU File Type"
echo "++++++++++++++++++++++++++++++++++++++++++++++++++++"
printf "\n"	


for domain in "same_domain" "different_domain"
	do
	domain_id=0
	temp_var="same_domain"
	
		if [ $domain = $temp_var ];
			then
			domain_id=10
		else
			domain_id=5

		fi 
		
	for rate in 10 20 40 80 100 200 500 1000
		do RMW_IMPLEMENTATION=$RMW ROS_DOMAIN_ID=$domain_id ros2 run cam_stream imu_publisher_relay_subscriber --ros-args -p filepath:=/root/imu.txt -p hz:=$rate -p domain:=$domain
		#/root/imu.txt
		echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
		echo "Experiment completed for IMU Type with rate- "$rate"hz for domain: "$domain" with domain id: "$domain_id
		echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
		done
done

 
exit
 
	
 	done
 done







































#/home/spaul/images/images_2mb /home/spaul/images/images_33kb /home/spaul/images/images_1mb /home/spaul/images/images_4mb /home/spaul/images/images_2mb /home/spaul/images/images_67kb /home/spaul/images/images_87kb /home/spaul/images/images_145kb /home/spaul/images/images_294kb /home/spaul/images/images_502kb
