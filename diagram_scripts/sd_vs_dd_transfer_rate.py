#!/usr/bin/env python3

import os
import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


print ("Started Calculation")
dd_transfer_rate_file_path = sys.argv[1]
sd_transfer_rate_file_path = sys.argv[2]
dds_name = sys.argv[3]
hz1="2hz"
hz2 = "4hz"


def get_transfer_rate( file_path):
    frequency_and_file_size_interprocess_map = {}
    dir_list = os.listdir(file_path)
    for dir in dir_list:
        temp_dir= os.path.join(file_path, dir)
        list_file= os.listdir(temp_dir)
        file_size_and_interprocessing_time_map = {}
        for file in list_file:
                    csv_file_path= os.path.abspath( temp_dir + os.sep +file)
                    filenamelist = file.split("_")
                    file_size = filenamelist[1]
                    temp_key = file_size.replace('kb','')
                    file_size_int = int(temp_key)
                    print("file size" + file_size)
                    df= pd.read_csv(csv_file_path)
                    start_time = df.iloc[0]['Subscribe_Time_Subscriber']
                    frame_count = len(df.index)
                    end_time = df.iloc[frame_count -1]['Publish_Time_From_Subscriber']
                    transfer_rate = ((frame_count -1)* file_size_int) / (float(end_time) - float(start_time ))
                    file_size_and_interprocessing_time_map[filenamelist[1]] = transfer_rate
        sorted_size_map = sort_size_map(file_size_and_interprocessing_time_map)
        frequency_and_file_size_interprocess_map[dir] = sorted_size_map
    return frequency_and_file_size_interprocess_map
                   

def prepare_binary_file_rtt(file_path):
    frequency_and_file_size_rtt_map = {}
    dir_list = os.listdir(file_path)
    for dir in dir_list:
        temp_dir= os.path.join(file_path, dir)
        list_file= os.listdir(temp_dir)
        file_size_and_rtt_map = {}
        for file in list_file:
            csv_file_path= os.path.abspath( temp_dir + os.sep +file)
            df= pd.read_csv(csv_file_path, encoding="utf-8")
            df['rtt'] = df.apply(lambda x: x['Subscribe_Time'] - x['Publish_Time'], axis=1)
            filenamelist = file.split("_")
            average_rtt = df['rtt'].mean()
            file_size_and_rtt_map[filenamelist[1]] = average_rtt
        sorted_size_map = sort_size_map(file_size_and_rtt_map)    
        frequency_and_file_size_rtt_map[dir]= sorted_size_map
    return frequency_and_file_size_rtt_map


    
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
    frequency_and_file_size_latency_map = {}
    for key in latency_map:
        latency_map_temp = latency_map.get(key)
        interprocess_time_map_temp = interprocess_time_map.get(key)

        result_map = {key: latency_map_temp[key] - interprocess_time_map_temp.get(key, 0)
                       for key in latency_map_temp.keys()}

        intermediate_map = convert_rtt_to_latency(result_map)
        #print(intermediate_map)
    
        sorted_result_map = sort_size_map(intermediate_map)
        frequency_and_file_size_latency_map[key] = sorted_result_map
    sorted_map = sort_unsorted_map(frequency_and_file_size_latency_map)
    return sorted_map


def sort_size_map(input_map):
    sizeconvertion = {'kb': 1024, 'mb' : 1024*1024}
    converted_size_map = {}
    for key in input_map:
        for x in [ 'kb', 'mb', 'gb', 'tb']:
            if x in key:
                size = float(key.replace(x, ""))
                size_in_bytes = size * sizeconvertion.get(x)
                converted_size_map[size_in_bytes] = input_map.get(key)
    key_in_bytes = list(converted_size_map.keys())
    key_in_bytes.sort()
    readable_bytes = {}
    for k in key_in_bytes:
       size = convert_bytes_to_readable_file_size(k)
       readable_bytes[size] = converted_size_map.get(k)
    return readable_bytes 


def convert_bytes_to_readable_file_size(size):
    for x in ['bytes', 'KB', 'MB', 'GB', 'TB']:
        if size < 1024.0:
            return "%3.1f %s" % (size, x)
        size /= 1024.0
    return size


def convert_rtt_to_latency(input_map):
    latency_map = {}
    for key in input_map.keys():
        latency_map[key] = float(input_map.get(key))/2
    return latency_map

        

def latency_transition_map(different_domain_latency, same_domain_latency):
    latency_transition_hz_map={}
    for hz in different_domain_latency:
        diff_domain_latency_temp = different_domain_latency.get(hz)
        same_domain_latency_temp = same_domain_latency.get(hz)
        # print("------------------")
        print(hz)
        # print(diff_domain_latency_temp)
        # print(same_domain_latency_temp)
        
        latency_transition_size_map ={}
        for size in diff_domain_latency_temp:
            print(size)
            print("+++++++++++++++++++++")
            latency_transition = diff_domain_latency_temp.get(size) - same_domain_latency_temp.get(size)
            # latency_transition = ((diff_domain_latency_temp.get(size) - same_domain_latency_temp.get(size))/same_domain_latency_temp.get(size)) * 100
            latency_transition_size_map[size] = latency_transition
            # print(latency_transition_size_map)
        latency_transition_hz_map[hz] = latency_transition_size_map
    
    return latency_transition_hz_map

def remove_enitty_from_map(input_list_of_key, input_map):
    shorten_map = {}
    for key in input_list_of_key:
        shorten_map[key] = input_map.get(key)
    return shorten_map




def main(args=None):

    different_domain_tr = get_transfer_rate(dd_transfer_rate_file_path)
    same_domain_tr = get_transfer_rate(sd_transfer_rate_file_path)


    
    input_list_key = [hz1, hz2]
    different_domain_tr1 = different_domain_tr
    same_domain_tr2 = same_domain_tr
    # latency_transition = latency_transition_map(different_domain_tr1, same_domain_latency_tr2)
    # diff_vs_same_domain ={}
    # diff_vs_same_domain["latency_shift"] = latency_transition
    dd_hz1= different_domain_tr1.get(hz1)
    sd_hz1= same_domain_tr2.get(hz1)
    dd_hz2= different_domain_tr1.get(hz2)
    sd_hz2= same_domain_tr2.get(hz2)
    # dd_hz3= different_domain_tr1.get(hz3)
    # sd_hz3= same_domain_tr2.get(hz3)
    
    same_domain_key_array = sd_hz1.keys()
    dd_hz1_value = dd_hz1.values()
    sd_hz1_value = sd_hz1.values()
    dd_hz2_value = dd_hz2.values()
    sd_hz2_value = sd_hz2.values()
    # dd_hz3_value = dd_hz3.values()
    # sd_hz3_value = sd_hz3.values()
    
    # print(dd_hz1_value )
    # print(sd_hz1_value )
    # print(dd_hz2_value )
    # print(sd_hz2_value )
    # print(dd_hz3_value )
    # print(sd_hz3_value )
    # fig = plt.figure()
    # ax = fig.add_axes([0,0,1,1])
    # result_map_6 = { "size": list(sd_map_12hz.keys()), "latency": list(sd_map_12hz.values())}
    # df4 = pd.DataFrame(result_map_4)
    N = 10
    plt.bar(2*np.arange(len(same_domain_key_array)) + 0.25, dd_hz1_value, color = '#1d960f', width = 0.25, label='different_domain-'+ hz1)
    plt.bar(2*np.arange(len(same_domain_key_array)) + 0.00, sd_hz1_value, color = '#39e6cf', width = 0.25, label= "same_domain-" + hz1)
    plt.bar(2*np.arange(len(same_domain_key_array)) + 0.75, dd_hz2_value, color = '#71b4eb', width = 0.25, label='different_domain-'+ hz2)
    plt.bar(2*np.arange(len(same_domain_key_array)) + 0.50, sd_hz2_value, color = '#bc71eb', width = 0.25, label= "same_domain-" + hz2)
    # plt.bar(2*np.arange(len(same_domain_key_array)) + 1.25, dd_hz3_value, color = '#eb7177', width = 0.25, label='different_domain-'+ hz3)
    # plt.bar(2*np.arange(len(same_domain_key_array)) + 1.00, sd_hz3_value, color = '#e6de6e', width = 0.25, label= "same_domain-" + hz3)
    plt.xticks(2*np.arange(len(same_domain_key_array)) + 1.25/2,  same_domain_key_array)
    plt.xlabel('File size')
    plt.ylabel('Transfer rate in KB/ms')
    plt.title('Same Domain Vs Different Domain transfer rate comparison for '+ dds_name + " for frequency- 2Hz and 4Hz")
    plt.legend()
    plt.show()

    # draw_graph_from_map(diff_vs_same_domain)




    

if __name__ == "__main__":
    main()