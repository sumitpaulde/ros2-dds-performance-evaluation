#!/usr/bin/env python3

import os
import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import ScalarFormatter
import matplotlib


print ("Started Calculation")
interprocessing_csv_path_dds1 = sys.argv[1]
interprocessing_csv_path_dds2 = sys.argv[2]
interprocessing_csv_path_dds3 = sys.argv[3]
imu_latency_path_dds1 = sys.argv[4]
imu_latency_path_dds2 = sys.argv[5]
imu_latency_path_dds3 = sys.argv[6]
dds_name = sys.argv[7]
hz1= "20hz"
hz2= "40hz"
hz3= "100hz"
hz4= "200hz"
hz5= "500hz"

label1 = "different_domain-" + hz1
label2 = "different_domain-" + hz2
label3 = "different_domain-" + hz3
label4= "same_domain-" + hz1
label5= "same_domain-" + hz2
label6= "same_domain-" + hz3


def get_interprocessing_time_map( file_path):
    frequency_and_file_size_interprocess_map = {}
    dir_list = os.listdir(file_path)
    for dir in dir_list:
        temp_dir= os.path.join(file_path, dir)
        list_file= os.listdir(temp_dir)
        file_size_and_interprocessing_time_map = {}
        for file in list_file:
                    csv_file_path= os.path.abspath( temp_dir + os.sep +file)
                    df= pd.read_csv(csv_file_path)
                    df['interprocesstime']= df.apply(lambda x: x['Publish_Time_From_Subscriber'] - x['Subscribe_Time_Subscriber'], axis=1)
                    # df['interprocesstime'] = df['Interprocess_Time']
                    filenamelist = file.split("_")
                    average_time = df['interprocesstime'].mean()
                    file_size_and_interprocessing_time_map[filenamelist[2]] = average_time
        frequency_and_file_size_interprocess_map[dir] = file_size_and_interprocessing_time_map
    print("From interprocess")
    print(frequency_and_file_size_interprocess_map)
    return frequency_and_file_size_interprocess_map
                   

def prepare_string_file_rtt(file_path):
    frequency_and_file_size_rtt_map = {}
    dir_list = os.listdir(file_path)
    for dir in dir_list:
        temp_dir= os.path.join(file_path, dir)
        list_file= os.listdir(temp_dir)
        file_size_and_rtt_map = {}
        for file in list_file:
            csv_file_path= os.path.abspath( temp_dir + os.sep +file)
            df= pd.read_csv(csv_file_path, encoding="utf-8")
            df['rtt'] = df.apply(lambda x: x['Subscribe_Timestamp_Publisher'] - x['Publish_Timestamp_Publisher'], axis=1)
            filenamelist = file.split("_")
            average_rtt = df['rtt'].mean()
            file_size_and_rtt_map[filenamelist[2]] = average_rtt
        frequency_and_file_size_rtt_map[dir]= file_size_and_rtt_map
        print("From IMU")
        print(frequency_and_file_size_rtt_map)
    return frequency_and_file_size_rtt_map


    
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

    
def rtt_to_latency(latency_map, interprocess_time_map):
    print(latency_map)
    print(interprocess_time_map)
    frequency_and_latency_map = {}
    for key in latency_map:
        latency_map_temp = latency_map.get(key)
        interprocess_time_map_temp = interprocess_time_map.get(key)

        result_map = {key: latency_map_temp[key] - interprocess_time_map_temp.get(key, 0)
                       for key in latency_map_temp.keys()}

        intermediate_map = convert_rtt_to_latency(result_map)
        #print(intermediate_map)
    
        sorted_result_map = sort_key_map(intermediate_map)
        frequency_and_latency_map[key] = sorted_result_map
    # sorted_map = sort_unsorted_map(frequency_and_file_size_latency_map)
    sorted_map = sort_unsorted_map(frequency_and_latency_map)
    print(sorted_map)
    return sorted_map


def convert_keys(input_map):
    key_values={}
    key_values["1338"] = "1.405KB"
    key_values["4"] = "0.004KB"
    key_values["91"] = "0.091KB"
    key_values["2569"] = "2.675KB"
    value_added_map={}
    for key in input_map:
        temp = key_values[key]
        value_added_map[temp] = input_map[key]
    return value_added_map


def convert_rtt_to_latency(input_map):
    latency_map = {}
    for key in input_map.keys():
        latency_map[key] = float(input_map.get(key))/2
    return latency_map



def remove_enitty_from_map(input_list_of_key, input_map):
    shorten_map = {}
    for key in input_list_of_key:
        print(key)
        shorten_map[key] = input_map.get(key)
    return shorten_map

def color_picker(hz):
    if hz == label1:
        return "#1adb71" 
    elif hz == label2:
        return "#19b9e6"
    elif hz == label3:
        return "#125c87"
    elif hz == label4:
        return "#e6c510"
    elif hz == label5:
        return "#6f19bf"
    elif hz == label6:
        return "#ba30bf" 
    elif hz == "8hz":
        return "#050c12" 
    elif hz == "10hz":
        return "#258bd9"
    elif hz == "12hz":
        return "#c75f0a"  
    elif hz == "15hz":
        return "#cf2d0c" 


def main(args=None):
    dds1_interprocess = (get_interprocessing_time_map(interprocessing_csv_path_dds1))
    dds1_rtt = prepare_string_file_rtt(imu_latency_path_dds1)
    dds1_latency = rtt_to_latency(dds1_rtt, dds1_interprocess)
    dds2_interprocess = (get_interprocessing_time_map(interprocessing_csv_path_dds2))
    dds2_rtt = prepare_string_file_rtt(imu_latency_path_dds1)
    dds2_latency = rtt_to_latency(dds2_rtt, dds2_interprocess)
    dds3_interprocess = (get_interprocessing_time_map(interprocessing_csv_path_dds3))
    dds3_rtt = prepare_string_file_rtt(imu_latency_path_dds3)
    dds3_latency = rtt_to_latency(dds3_rtt, dds3_interprocess)


    
    input_list_key = [hz1, hz2, hz3, hz4]
    dds1_shorten_latency = remove_enitty_from_map(input_list_key, dds1_latency)
    dds2_latency_shorten = remove_enitty_from_map(input_list_key, dds2_latency)
    dds3_latency_shorten = remove_enitty_from_map(input_list_key, dds3_latency)
    

    dd_hz1= dds1_shorten_latency.get(hz1)
    sd_hz1= dds2_latency_shorten.get(hz1)
    dd_hz2= dds1_shorten_latency.get(hz2)
    sd_hz2= dds2_latency_shorten.get(hz2)
    dd_hz3= dds1_shorten_latency.get(hz3)
    sd_hz3= dds2_latency_shorten.get(hz3)
    dd_hz4= dds1_shorten_latency.get(hz4)
    sd_hz4= dds2_latency_shorten.get(hz4)
    dds3_hz1= dds3_latency_shorten.get(hz1)
    dds3_hz2= dds2_latency_shorten.get(hz2)
    dds3_hz3= dds3_latency_shorten.get(hz3)
    dds3_hz4= dds2_latency_shorten.get(hz4)
    
    # same_domain_key_array = sd_hz1.keys()
    dd_hz1_value = dd_hz1.values()
    sd_hz1_value = sd_hz1.values()
    dd_hz2_value = dd_hz2.values()
    sd_hz2_value = sd_hz2.values()
    dd_hz3_value = dd_hz3.values()
    sd_hz3_value = sd_hz3.values()
    dd_hz4_value = dd_hz4.values()
    sd_hz4_value = sd_hz4.values()
    dds3_hz1_value  = dds3_hz1.values()
    dds3_hz2_value  = dds3_hz2.values()
    dds3_hz3_value  = dds3_hz3.values()
    dds3_hz4_value  = dds3_hz4.values()
    sd1 = list(sd_hz1_value )
    sd2 = list(sd_hz2_value)
    sd3 = list(sd_hz3_value)
    sd4 = list(sd_hz4_value)
    dd1 = list(dd_hz1_value)
    dd2 = list(dd_hz2_value)
    dd3 = list(dd_hz3_value)
    dd4 = list(dd_hz4_value)
    dds3_v1 = list(dds3_hz1_value)
    dds3_v2 = list(dds3_hz2_value)
    dds3_v3 = list(dds3_hz3_value)
    dds3_v4 = list(dds3_hz4_value)
    sd_array = [sd1[0], sd2[0], sd3[0], sd4[0] ]
    dd_array = [dd1[0], dd2[0], dd3[0], dd4[0]]
    dds3_array = [dds3_v1[0], dds3_v2[0],dds3_v3[0],dds3_v4[0]]


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
    plt.bar(ticks_value +   0.00, sd_array, color = '#39e6cf', width = 0.25, label= "cyclone_dds")
    plt.bar( ticks_value +0.25, dd_array, color = '#71b4eb', width = 0.25, label='eprosima_dds')
    plt.bar( ticks_value +0.50, dds3_array, color = '#8ce07e', width = 0.25, label='rti_connext_dds')
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
    plt.ylabel('Latency in ms')
    plt.title('Different DDS IMU latency comparison for '+ dds_name )
    plt.legend(bbox_to_anchor=(1,1), loc='upper left', borderaxespad=0)
    plt.show()

    # draw_graph_from_map(diff_vs_same_domain)


    

if __name__ == "__main__":
    main()