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

# commented out the bellow block as we will run the test for same and different domain simultaneously.
: '
v1=$s
v2=$d


 while :
	do
	echo "Please select a DDS-Implementation to RUN the experiments"
	echo "Select the DDS-Implementation according to the Docker Image."
	echo "++++++++++++++++++++++++++++++++++++++++++++++++++++"
	printf "\n"
	echo "s. Experiment for SAME-DOMAIN"
	echo "d. Experiment for DIFFERENT-DOMAIN"
	echo -n "Type s or d :"
	read -n 1 -t 30 i
	printf "\n"
		case $i in
	 		s* )    domain="same_domain";;
 
			d* )    domain="different_domain";;
 
 
			* )     echo "Please choose between s or d.";;
		esac
		

echo "---------------------------------------------"
echo "Experiment setup for : "$domain "selected"
echo "---------------------------------------------"
'

echo "ROS2 Performance Analysis Experiment Started"
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

echo "++++++++++++++++++++++++++++++++++++++++++++++++++++"
echo "+ Experiment Running for Binary Files +"
echo "++++++++++++++++++++++++++++++++++++++++++++++++++++"

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
	
		for filepath in /root/images/images_33kb /root/images/images_67kb /root/images/images_87kb /root/images/images_145kb /root/images/images_294kb /root/images/images_502kb /root/images/images_1mb /root/images/images_2mb /root/images/images_4mb 
	do

			for rate in 1 2 4 8 10 12 15 20 100 1000
			do RMW_IMPLEMENTATION=$RMW ROS_DOMAIN_ID=$domain_id ros2 run cam_stream binary_publisher_relay_subscriber --ros-args -p filepath:=$filepath -p hz:=$rate -p domain:=$domain
			#
			printf "\n"
			printf "\n"
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
			echo "Experiment done for "$filepath" and rate "$rate"hz for domain: "$domain" with domain id: "$domain_id
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
			done
		done
		
	done	


 
exit
 
	
 	done
 done







































#/home/spaul/images/images_2mb /home/spaul/images/images_33kb /home/spaul/images/images_1mb /home/spaul/images/images_4mb /home/spaul/images/images_2mb /home/spaul/images/images_67kb /home/spaul/images/images_87kb /home/spaul/images/images_145kb /home/spaul/images/images_294kb /home/spaul/images/images_502kb
