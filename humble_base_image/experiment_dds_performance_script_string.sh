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


echo "++++++++++++++++++++++++++++++++++++++++++++++++++++"
echo "Experiment Starting for String File Type"
echo "++++++++++++++++++++++++++++++++++++++++++++++++++++"
printf "\n"

# declare +i
 char_4="hare"
 char_91="Hare Krishna Hare Krishna Krishna Krishna Hare Hare Hare Rama Hare Rama Rama Rama Hare Hare"
char_1342="The word dharma-kṣetra (a place where religious rituals are performed) is significant because, on the Battlefield of Kurukṣetra, the Supreme Personality of Godhead was present on the side of Arjuna. Dhṛtarāṣṭra, the father of the Kurus, was highly doubtful about the possibility of his sons’ ultimate victory. In his doubt, he inquired from his secretary Sañjaya,What did they do He was confident that both his sons and the sons of his younger brother Pāṇḍu were assembled in that Field of Kurukṣetra for a determined engagement of the war. Still, his inquiry is significant. He did not want a compromise between the cousins and brothers, and he wanted to be sure of the fate of his sons on the battlefield. Because the battle was arranged to be fought at Kurukṣetra, which is mentioned elsewhere in the Vedas as a place of worship – even for the denizens of heaven – Dhṛtarāṣṭra became very fearful about the influence of the holy place on the outcome of the battle. He knew very well that this would influence Arjuna and the sons of Pāṇḍu favorably, because by nature they were all virtuous. Sañjaya was a student of Vyāsa, and therefore, by the mercy of Vyāsa, Sañjaya was able to envision the Battlefield of Kurukṣetra even while he was in the room of Dhṛtarāṣṭra. And so, Dhṛtarāṣṭra asked him about the situation on the battlefield."
 char_2571="Lord Caitanya met the two brothers Dabira Khāsa and Sākara Mallika in a village known as Rāmakeli in the district of Maldah, and after that meeting the brothers decided to retire from government service and join Lord Caitanya. Dabira Khāsa, who was later to become Rūpa Gosvāmī, retired from his post and collected all the money he had accumulated during his service. It is described in the Caitanya-caritāmṛta that his accumulated savings in gold coins equaled millions of dollars and filled a large boat. He divided the money in a very exemplary manner, which should be followed by devotees in particular and by humanity in general. Fifty percent of his accumulated wealth was distributed to the Kṛṣṇa conscious persons, namely the brāhmaṇas and the Vaiṣṇavas; twenty-five percent was distributed to relatives; and twenty-five percent was kept against emergency expenditures and personal difficulties. Later on, when Sākara Mallika also proposed to retire, the Nawab was very much agitated and put him into jail. But Sākara Mallika, who was later to become Śrīla Sanātana Gosvāmī, took advantage of his brother’s personal money, which had been deposited with a village banker, and escaped from the prison of Hussain Shah. In this way both brothers joined Lord Caitanya Mahāprabhu.Rūpa Gosvāmī later met Lord Caitanya at Prayāga (Allahabad, India), and on the Daśāśvamedha bathing ghāṭa of that holy city the Lord instructed him continually for ten days. The Lord particularly instructed Rūpa Gosvāmī on the science of Kṛṣṇa consciousness. These teachings of Lord Caitanya to Śrīla Rūpa Gosvāmī Prabhupāda are narrated in our book Teachings of Lord Caitanya.Later, Śrīla Rūpa Gosvāmī Prabhupāda elaborated the teachings of the Lord with profound knowledge of revealed scriptures and authoritative references from various Vedic literatures. Śrīla Śrīnivāsa Ācārya describes in his prayers to the Six Gosvāmīs that they were all highly learned scholars, not only in Sanskrit but also in foreign languages such as Persian and Arabic. They very scrutinizingly studied all the Vedic scriptures in order to establish the movement of Caitanya Mahāprabhu on the authorized principles of Vedic knowledge. The present Kṛṣṇa consciousness movement is also based on the authority of Śrīla Rūpa Gosvāmī Prabhupāda. We are therefore generally known as rūpānugas, or followers in the footsteps of Śrīla Rūpa Gosvāmī Prabhupāda. It is only for our guidance that Śrīla Rūpa Gosvāmī prepared his book Bhakti-rasāmṛta-sindhu, which is now presented in the form of The Nectar of Devotion."

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
			do RMW_IMPLEMENTATION=$RMW ROS_DOMAIN_ID=$domain_id ros2 run cam_stream string_publisher_relay_subscriber --ros-args -p input:="$char_4" -p hz:=$rate -p domain:=$domain
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
			echo "Experiment completed for String Type- "$rate"hz for domain: "$domain" with domain id: "$domain_id
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
		done
		
		for rate in 10 20 40 80 100 200 500 1000
			do RMW_IMPLEMENTATION=$RMW ROS_DOMAIN_ID=$domain_id ros2 run cam_stream string_publisher_relay_subscriber --ros-args -p input:="$char_91" -p hz:=$rate -p domain:=$domain
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
			echo "Experiment completed for String Type- "$rate"hz for domain: "$domain" with domain id: "$domain_id
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
		done
		
		for rate in 10 20 40 80 100 200 500 1000
			do RMW_IMPLEMENTATION=$RMW ROS_DOMAIN_ID=$domain_id ros2 run cam_stream string_publisher_relay_subscriber --ros-args -p input:="$char_1342" -p hz:=$rate -p domain:=$domain
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
			echo "Experiment completed for String Type- "$rate"hz for domain: "$domain" with domain id: "$domain_id
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
		done
		
		for rate in 10 20 40 80 100 200 500 1000
			do RMW_IMPLEMENTATION=$RMW ROS_DOMAIN_ID=$domain_id ros2 run cam_stream string_publisher_relay_subscriber --ros-args -p input:="$char_2571" -p hz:=$rate -p domain:=$domain
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
			echo "Experiment completed for String Type- "$rate"hz for domain: "$domain" with domain id: "$domain_id
			echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
		done
	done
	

exit
 
	
 	done
 done







































#/home/spaul/images/images_2mb /home/spaul/images/images_33kb /home/spaul/images/images_1mb /home/spaul/images/images_4mb /home/spaul/images/images_2mb /home/spaul/images/images_67kb /home/spaul/images/images_87kb /home/spaul/images/images_145kb /home/spaul/images/images_294kb /home/spaul/images/images_502kb
