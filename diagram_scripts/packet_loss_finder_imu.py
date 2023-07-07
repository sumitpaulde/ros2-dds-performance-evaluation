#!/usr/bin/env python3

import os
import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import ScalarFormatter
import matplotlib


print ("Started Calculation")
file_path_different_domain= sys.argv[1]
file_path_same_domain = sys.argv[2]
dds_name = sys.argv[3]
hz1= "20hz"
hz2= "40hz"
hz3= "100hz"
hz4= "200hz"


label1 = "different_domain-" + hz1
label2 = "different_domain-" + hz2
label3 = "different_domain-" + hz3
label4= "same_domain-" + hz1
label5= "same_domain-" + hz2
label6= "same_domain-" + hz3


def packet_loss_map_provider(file_path):
    # os.chdir(file_path)
    frequency_and_file_size_packet_loss_map = {}
    dir_list = os.listdir(file_path)
    for dir in dir_list:
        temp_dir= os.path.join(file_path, dir)
        list_file= os.listdir(temp_dir)
        file_size_and_packet_loss_map = {}
        for file in list_file:
            csv_file_path= os.path.abspath( temp_dir + os.sep +file)
            df= pd.read_csv(csv_file_path, encoding="utf-8")
            df['packet_loss'] = df.apply(lambda x: ((( x['Published_Number_of_Frame'] - x['Received_Number_Of_Frame'])/x['Published_Number_of_Frame']) * 100 ), axis=1)
            filenamelist = file.split("_")
            average_packet_loss = df['packet_loss'].mean()
            average_packet_received = 100 - average_packet_loss
            file_size_and_packet_loss_map[filenamelist[1]] = average_packet_received
            
        frequency_and_file_size_packet_loss_map[dir]= file_size_and_packet_loss_map
    print(file_size_and_packet_loss_map)
    frequency_and_file_size_packet_loss_map_sorted = sort_unsorted_map(frequency_and_file_size_packet_loss_map)
    return frequency_and_file_size_packet_loss_map_sorted
                   




    
def sort_key_map(input_map):
    unsorted_Key_int = {int(k):float(v) for k,v in input_map.items()}
    sorted_key=[]
    unsortedKeys = list(unsorted_Key_int.keys())
    unsortedKeys.sort()
    sorted_dict = {i: unsorted_Key_int[i] for i in unsortedKeys}
    sorted_dict_output = {str(k):float(v) for k,v in sorted_dict.items()}
    # converted_key_map = convert_keys(sorted_dict_output)
    # print(new_dict)
    return sorted_dict_output


def sort_unsorted_map(input_map):
    Keys_unsorted = list(input_map.keys())
    sorted_key=[]
    print(Keys_unsorted)
    for key in Keys_unsorted:
        temp_key = key.replace('hz','')
        temp_key_int = int(temp_key)
        sorted_key.append(temp_key_int)
        
    sorted_key.sort()
    # print(sorted_key)
    Keys = []
    for key in sorted_key:
        key = str(key) +"hz"
        Keys.append(key) 
    new_dict = {key: input_map[key] for key in Keys}
    # print(new_dict)
    return new_dict

    









def remove_enitty_from_map(input_list_of_key, input_map):
    shorten_map = {}
    for key in input_list_of_key:
        print(key)
        shorten_map[key] = input_map.get(key)
    return shorten_map




def main(args=None):

    dd_pkt_loss = (dict(packet_loss_map_provider(file_path_different_domain)))
    sd_pkt_loss = (dict(packet_loss_map_provider(file_path_same_domain)))

    print(dds_name )
    print(20*"-")
    print("same_domain_latency")
    print(sd_pkt_loss)
    print(20*"-")
    print(20*"-")
    print("different_domain_latency")
    print(dd_pkt_loss)


    
    input_list_key = [hz1, hz2, hz3, hz4]
    different_domain_latency_shorten = remove_enitty_from_map(input_list_key, dd_pkt_loss)
    same_domain_latency_shorten = remove_enitty_from_map(input_list_key, sd_pkt_loss)
    print(same_domain_latency_shorten)
    dd_hz1= different_domain_latency_shorten.get(hz1)
    sd_hz1= same_domain_latency_shorten.get(hz1)
    dd_hz2= different_domain_latency_shorten.get(hz2)
    sd_hz2= same_domain_latency_shorten.get(hz2)
    dd_hz3= different_domain_latency_shorten.get(hz3)
    sd_hz3= same_domain_latency_shorten.get(hz3)
    dd_hz4= different_domain_latency_shorten.get(hz4)
    sd_hz4= same_domain_latency_shorten.get(hz4)
    # dd_hz5= different_domain_latency_shorten.get(hz5)
    # sd_hz5= same_domain_latency_shorten.get(hz5)
    
    # same_domain_key_array = sd_hz1.keys()
    dd_hz1_value = dd_hz1.values()
    sd_hz1_value = sd_hz1.values()
    dd_hz2_value = dd_hz2.values()
    sd_hz2_value = sd_hz2.values()
    dd_hz3_value = dd_hz3.values()
    sd_hz3_value = sd_hz3.values()
    dd_hz4_value = dd_hz4.values()
    sd_hz4_value = sd_hz4.values()
    # dd_hz5_value = dd_hz5.values()
    # sd_hz5_value = sd_hz5.values()
    sd1 = list(sd_hz1_value )
    sd2 = list(sd_hz2_value)
    sd3 = list(sd_hz3_value)
    sd4 = list(sd_hz4_value)
    dd1 = list(dd_hz1_value)
    dd2 = list(dd_hz2_value)
    dd3 = list(dd_hz3_value)
    dd4 = list(dd_hz4_value)
    # sd5 = list(sd_hz5_value)
    # dd5 = list(dd_hz5_value)
    sd_array = [sd1[0], sd2[0], sd3[0], sd4[0]]
    dd_array = [dd1[0], dd2[0], dd3[0], dd4[0]]


    # # print(dd_hz2_value )
    # # print(sd_hz2_value )
    # # print(dd_hz3_value )
    # # print(sd_hz3_value )
    # # fig = plt.figure()
    # # ax = fig.add_axes([0,0,1,1])
    # # result_map_6 = { "size": list(sd_map_12hz.keys()), "latency": list(sd_map_12hz.values())}
    # # df4 = pd.DataFrame(result_map_4)
    # N = 10
    # 3*np.arange(len(same_domain_key_array)) 
    # sd_hz1_value = same_domain_latency_shorten.values()
    # dd_hz1_value = different_domain_latency_shorten.values()
    # key_array = same_domain_latency_shorten.keys()

    
    # same_domain_key_array = sd_hz1.keys()
    ticks_value = np.arange(4) 
    print(dd_hz1)
    fig = plt.figure(figsize=(14, 8))
    plt.bar(ticks_value +   0.00, sd_array, color = '#39e6cf', width = 0.25, label= "s_domain")
    plt.bar( ticks_value +0.25, dd_array, color = '#71b4eb', width = 0.25, label='d_domain')
    # plt.bar(ticks_value+ 0.50, sd_hz2_value, color = '#bc71eb', width = 0.25, label= "s_domain-" + hz2)
    # plt.bar( 0.75, dd_hz2_value, color = '#71b4eb', width = 0.25, label='d_domain-'+ hz2)
    # plt.bar(1.00, sd_hz3_value, color = '#e6de6e', width = 0.25, label= "s_domain-" + hz3)
    # plt.bar(  1.25, dd_hz3_value, color = '#eb7177', width = 0.25, label='d_domain-'+ hz3)
    # plt.bar( 1.50, sd_hz4_value, color = '#4812c7', width = 0.25, label= "s-domain" + hz4)
    # plt.bar( 1.75, dd_hz4_value, color = '#8ce07e', width = 0.25, label='d_domain-'+ hz4)
    # plt.gca().yaxis.set(major_formatter=ScalarFormatter(), minor_formatter=ScalarFormatter());
    # formatter = matplotlib.ticker.ScalarFormatter()
    # formatter.set_powerlimits((-1,1))
    # plt.gca().yaxis.set_major_formatter(formatter)
    # plt.gca().yaxis.set_minor_locator(matplotlib.ticker.NullLocator())
    plt.xticks(ticks_value + 0.15 , input_list_key   )
    plt.xlabel('Publisher Frequency')
    plt.ylabel('Packet received measurement')
    plt.title('Same Domain Vs Different Domain IMU packet transfer comparison for '+ dds_name )
    plt.legend(bbox_to_anchor=(1,1), loc='upper left', borderaxespad=0)
    plt.show()

    # draw_graph_from_map(diff_vs_same_domain)


    

if __name__ == "__main__":
    main()