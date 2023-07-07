#!/usr/bin/env python3

import os
import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np



print ("Started Calculation")
file_path_different_domain = sys.argv[1]
file_path_same_domain = sys.argv[2]
dds_name = sys.argv[3]

hz1= "2hz"
hz2= "4hz"
hz3= "10hz"
hz4= "20hz"
# hz2= "2hz"
# hz3= "4hz"

temp_map= []

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
    frequency_and_file_size_packet_loss_map_sorted = prepare_unsorted_map(frequency_and_file_size_packet_loss_map)
    return frequency_and_file_size_packet_loss_map_sorted
        #return frequency_and_file_size_packet_loss_map


    
def prepare_unsorted_map(input_map):
    sorted_size_map = {}
    for freq in input_map:
         sorted_map = sort_size_map(input_map.get(freq))
         sorted_size_map[freq] = sorted_map
    Keys_unsorted = list(sorted_size_map.keys())
    sorted_key=[]
    print(Keys_unsorted)
    for key in Keys_unsorted:
        temp_key = key.replace('hz','')
        temp_key_int = int(temp_key)
        sorted_key.append(temp_key_int)
        
    sorted_key.sort()
    print(sorted_key)
    Keys = []
    for key in sorted_key:
        key = str(key) +"hz"
        Keys.append(key) 
    new_dict = {key: sorted_size_map[key] for key in Keys}
    # print(new_dict)
    return new_dict

    


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

def color_picker(hz):
    if hz == "20hz":
        return "#6f19bf"
    elif hz == "100hz":
        return "#19b9e6"
    elif hz == "1000hz":
        return "#125c87"
    elif hz == "1hz":
        return "#e6c510"
    elif hz == "2hz":
        return "#258bd9"
    elif hz == "4hz":
        return "#ba30bf" 
    elif hz == "8hz":
        return "#050c12" 
    elif hz == "10hz":
        return "#1adb71"  
    elif hz == "12hz":
        return "#c75f0a"  
    elif hz == "15hz":
        return "#cf2d0c"     


def draw_graph_from_map(data_map):
    for domain in data_map:
        sorted_map = data_map.get(domain)

        for hz in sorted_map:
            result_map = sorted_map.get(hz)
            #print(hz)
            # print(result_map)
            # print("-------------------------")

            result_map_for_data_frame = { "size": list(result_map.keys()), "packet_received": list(result_map.values())}
            df = pd.DataFrame(result_map_for_data_frame)
            plt.plot(df["size"], df["packet_received"], color=color_picker(hz), marker='.', label = domain +"--"+hz)
            plt.legend(bbox_to_anchor=(1,1), loc='upper left', borderaxespad=0)
            plt.title(dds_name)
            plt.xlabel("File size in bytes")
            plt.ylabel("Packet Received in Percentage")
        plt.show()


def remove_enitty_from_map(input_list_of_key, input_map):
    shorten_map = {}
    for key in input_list_of_key:
        shorten_map[key] = input_map.get(key)
    return shorten_map



def main(args=None):
    same_domain_map = (dict(packet_loss_map_provider(file_path_same_domain)))
    different_domain_map = (dict(packet_loss_map_provider(file_path_different_domain)))
    # print(same_domain_map)
    # data_map ={}
    # data_map["same_domain"] = same_domain_map
    # data_map["different_domain"] = different_domain_map
    # draw_graph_from_map(data_map)

    input_list_key = [hz1, hz2, hz3, hz4]
    dd_packet_loss = remove_enitty_from_map(input_list_key, different_domain_map)
    sd_packet_loss = remove_enitty_from_map(input_list_key, same_domain_map)
    dd_hz1= dd_packet_loss.get(hz1)
    sd_hz1= sd_packet_loss.get(hz1)
    dd_hz2= dd_packet_loss.get(hz2)
    sd_hz2= sd_packet_loss.get(hz2)
    dd_hz3= dd_packet_loss.get(hz3)
    sd_hz3= sd_packet_loss.get(hz3)
    dd_hz4= dd_packet_loss.get(hz4)
    sd_hz4= sd_packet_loss.get(hz4)
    # dd_hz5= dd_packet_loss.get(hz5)
    # sd_hz5= sd_packet_loss.get(hz5)

    same_domain_key_array = sd_hz1.keys()
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

    ticks_value = 2.5*np.arange(len(same_domain_key_array)) 
    fig = plt.figure(figsize=(14, 8))
    plt.bar(ticks_value  + 0.00  , sd_hz1_value, color = '#39e6cf', width = 0.25, label= "s_domain-" + hz1)
    plt.bar(ticks_value  + 0.25, dd_hz1_value, color = '#1d960f', width = 0.25, label='d_domain-'+ hz1)
    plt.bar(ticks_value  + 0.50, sd_hz2_value, color = '#bc71eb', width = 0.25, label= "s_domain-" + hz2)
    plt.bar(ticks_value  + 0.75, dd_hz2_value, color = '#71b4eb', width = 0.25, label='d_domain-'+ hz2)
    plt.bar(ticks_value  + 1.00, sd_hz3_value, color = '#e6de6e', width = 0.25, label= "s_domain-" + hz3)
    plt.bar(ticks_value  + 1.25, dd_hz3_value, color = '#eb7177', width = 0.25, label='d_domain-'+ hz3)
    plt.bar(ticks_value  + 1.50, sd_hz4_value, color = '#4812c7', width = 0.25, label= "s-domain" + hz4)
    plt.bar(ticks_value  + 1.75, dd_hz4_value, color = '#8ce07e', width = 0.25, label='d_domain-'+ hz4)
    # plt.bar(ticks_value  + 2.00, sd_hz5_value, color = '#e80e0e', width = 0.25, label= "s-domain" + hz5)
    # plt.bar(ticks_value  + 2.25, dd_hz5_value, color = '#1b1be0', width = 0.25, label='d_domain-'+ hz5)
    plt.xticks(ticks_value  + 1.25/2,  same_domain_key_array)
    plt.xlabel('File size')
    plt.ylabel('Packet Received in Percentage')
    plt.title('Same Domain Vs Different Domain Packet Received comparison for '+ dds_name )
    plt.legend(bbox_to_anchor=(1,1), loc='upper left', borderaxespad=0)
    plt.show()


if __name__ == "__main__":
    main()