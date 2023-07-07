#!/usr/bin/env python3

import os
import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


print ("Started Calculation")
interprocessing_csv_path_dds1 = sys.argv[1]
interprocessing_csv_path_dds2 = sys.argv[2]
interprocessing_csv_path_dds3 = sys.argv[3]
image_latency_path_dds1 = sys.argv[4]
image_latency_path_dds2 = sys.argv[5]
image_latency_path_dds3 = sys.argv[6]

# for example diferent dds different domain wired connection
experiment_name = sys.argv[7]
hz1= sys.argv[8]
hz2= sys.argv[9]
hz3= sys.argv[10]

dds1 = "eclipse_cyclone"
dds2 = "eprosima_dds"
dds3 = "rti_connext"

label1_1 = dds1 + "-" + hz1
label1_2 = dds1 + "-" + hz2
label1_3 = dds1 + "-" + hz3
label2_1=  dds2+ "-" + hz1
label2_2= dds2 + "-" + hz2
label2_3= dds2 + "-" + hz3
label3_1=  dds3+ "-" + hz1
label3_2= dds3 + "-" + hz2
label3_3= dds3 + "-" + hz3




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
                    file_size_and_interprocessing_time_map[filenamelist[1]] = average_time
        frequency_and_file_size_interprocess_map[dir] = file_size_and_interprocessing_time_map
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
        frequency_and_file_size_rtt_map[dir]= file_size_and_rtt_map
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

def color_picker(hz):
    if hz == label1_1:
        return "#258bd9"
    elif hz == label1_2:
        return "#19b9e6"
    elif hz == label1_3:
        return "#125c87"
    elif hz == label2_1:
        return "#e6c510"
    elif hz == label2_2:
        return "#6f19bf"
    elif hz == label2_3:
        return "#ba30bf" 
    elif hz == label3_1:
        return "#050c12" 
    elif hz == label3_2:
        return "#1adb71"  
    elif hz == label3_3:
        return "#c75f0a"  
    elif hz == "15hz":
        return "#cf2d0c"     



def remove_enitty_from_map(input_list_of_key, input_map):
    shorten_map = {}
    for key in input_list_of_key:
        shorten_map[key] = input_map.get(key)
    return shorten_map



def main(args=None):
    interprocess_dds1 = (get_interprocessing_time_map(interprocessing_csv_path_dds1))
    rtt_dds1 = prepare_binary_file_rtt(image_latency_path_dds1)
    latency_dds1 = rtt_to_latency(rtt_dds1, interprocess_dds1)

    interprocess_dds2 = (get_interprocessing_time_map(interprocessing_csv_path_dds2))
    rtt_dds2 = prepare_binary_file_rtt(image_latency_path_dds2)
    latency_dds2 = rtt_to_latency(rtt_dds2, interprocess_dds2)

    interprocess_dds3 = (get_interprocessing_time_map(interprocessing_csv_path_dds3))
    rtt_dds3 = prepare_binary_file_rtt(image_latency_path_dds3)
    latency_dds3 = rtt_to_latency(rtt_dds3, interprocess_dds3)


    input_list_key = [hz1, hz2, hz3]


    dds1_latency_shorten = remove_enitty_from_map(input_list_key, latency_dds1)
    dds2_latency_shorten = remove_enitty_from_map(input_list_key, latency_dds2)
    dds3_latency_shorten = remove_enitty_from_map(input_list_key, latency_dds3)

    dds1_map_hz1 = dds1_latency_shorten.get(hz1)
    dds1_map_hz2 = dds1_latency_shorten.get(hz2)
    dds1_map_hz3 = dds1_latency_shorten.get(hz3)
    dds1_hz1_value = dds1_map_hz1.values()
    dds1_hz2_value = dds1_map_hz2.values()
    dds1_hz3_value = dds1_map_hz3.values()

    # result_map_1 = { "size": list(dds1_map_hz1.keys()), "latency": list(dds1_map_hz1.values())}
    # result_map_2 = { "size": list(dds1_map_hz2.keys()), "latency": list(dds1_map_hz2.values())}
    # result_map_3 = { "size": list(dd1_map_hz3.keys()), "latency": list(dd1_map_hz3.values())}
    # df1 = pd.DataFrame(result_map_1)
    # df2 = pd.DataFrame(result_map_2)
    # df3 = pd.DataFrame(result_map_3)

    dds2_map_hz1 = dds2_latency_shorten.get(hz1)
    dds2_map_hz2 = dds2_latency_shorten.get(hz2)
    dds2_map_hz3 = dds2_latency_shorten.get(hz3)
    dds2_hz1_value = dds2_map_hz1.values()
    dds2_hz2_value = dds2_map_hz2.values()
    dds2_hz3_value = dds2_map_hz3.values()

    # result_map_4 = { "size": list(dds2_map_hz1.keys()), "latency": list(dds2_map_hz1.values())}
    # result_map_5 = { "size": list(dds2_map_hz2.keys()), "latency": list(dds2_map_hz2.values())}
    # result_map_6 = { "size": list(dds2_map_hz3.keys()), "latency": list(dds2_map_hz3.values())}
    # df4 = pd.DataFrame(result_map_4)
    # df5 = pd.DataFrame(result_map_5)
    # df6 = pd.DataFrame(result_map_6)

    dds3_map_hz1 = dds3_latency_shorten.get(hz1)
    dds3_map_hz2 = dds3_latency_shorten.get(hz2)
    dds3_map_hz3 = dds3_latency_shorten.get(hz3)
    dds3_hz1_value = dds3_map_hz1.values()
    dds3_hz2_value = dds3_map_hz2.values()
    dds3_hz3_value = dds3_map_hz3.values()

    # result_map_7 = { "size": list(dds3_map_hz1.keys()), "latency": list(dds3_map_hz1.values())}
    # result_map_8 = { "size": list(dds3_map_hz2.keys()), "latency": list(dds3_map_hz2.values())}
    # result_map_9 = { "size": list(dds3_map_hz3.keys()), "latency": list(dds3_map_hz3.values())}
    # df7 = pd.DataFrame(result_map_7)
    # df8 = pd.DataFrame(result_map_8)
    # df9 = pd.DataFrame(result_map_9)
    input_key_array = dds1_map_hz1.keys()

    plt.bar(3*np.arange(len(input_key_array)) + 0.00, dds1_hz1_value, color = '#1d960f', width = 0.25, label= label1_1)
    plt.bar(3*np.arange(len(input_key_array)) + 0.25, dds2_hz1_value, color = '#39e6cf', width = 0.25, label= label2_1)
    plt.bar(3*np.arange(len(input_key_array)) + 0.50, dds3_hz1_value, color = '#71b4eb', width = 0.25, label=label3_1)
    plt.bar(3*np.arange(len(input_key_array)) + 0.75, dds1_hz2_value, color = '#bc71eb', width = 0.25, label= label1_2)
    plt.bar(3*np.arange(len(input_key_array)) + 1.00, dds2_hz2_value, color = '#eb7177', width = 0.25, label= label2_2)
    plt.bar(3*np.arange(len(input_key_array)) + 1.25, dds3_hz2_value, color = '#e6de6e', width = 0.25, label= label3_2)
    plt.bar(3*np.arange(len(input_key_array)) + 1.50, dds1_hz3_value, color = '#0b51b3', width = 0.25, label= label1_3)
    plt.bar(3*np.arange(len(input_key_array)) + 1.75, dds2_hz3_value, color = '#e09119', width = 0.25, label= label2_3)
    plt.bar(3*np.arange(len(input_key_array)) + 2.00, dds3_hz3_value, color = '#c2e66a', width = 0.25, label= label3_3)
    plt.xticks(3*np.arange(len(input_key_array)) + 1.25/2,  input_key_array)
    plt.xlabel('File size')
    plt.ylabel('Latency in ms')
    plt.title('Different DDS binary latency comparison for '+ experiment_name + " for frequency- " + hz1 +" " + hz2 + " " + hz3)
    plt.legend()
    plt.show()










    # plt.plot(df1["size"], df1["latency"], color=color_picker(label1), marker='.', label = label1)
    # plt.plot(df2["size"], df2["latency"], color=color_picker(label2), marker='.', label = label2)
    # plt.plot(df3["size"], df3["latency"], color=color_picker(label3), marker='.', label= label3)
    # plt.plot(df4["size"], df4["latency"], color=color_picker(label4), marker='.', label = label4)
    # plt.plot(df5["size"], df5["latency"], color=color_picker(label5), marker='.', label = label5)
    # plt.plot(df6["size"], df6["latency"], color=color_picker(label6), marker='.', label= label6)
    # plt.plot(df7["size"], df7["latency"], color=color_picker(label7), marker='.', label = label7)
    # plt.plot(df8["size"], df8["latency"], color=color_picker(label8), marker='.', label = label8)
    # plt.plot(df9["size"], df9["latency"], color=color_picker(label9), marker='.', label= label9)
    # plt.title(title)
    # plt.xlabel("Binary file size")
    # plt.ylabel("Latency in ms")
    # plt.legend(bbox_to_anchor=(1,1), loc='upper left', borderaxespad=0)
    # plt.show()

    

if __name__ == "__main__":
    main()