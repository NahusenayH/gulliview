import os
import pandas as pd
from statistics import mean, stdev
import matplotlib.pyplot as plt

import locale
locale.setlocale(locale.LC_NUMERIC, "C")
plt.rcdefaults()
# Tell matplotlib to use the locale we set above
plt.rcParams['axes.formatter.use_locale'] = True

"""
Unfinished function for visualizing samples of a static log.
"""
tmp = os.listdir(os.getcwd())
mode = input("s for static, m for moving, t for timing: ")

if mode == "s":
    csvs = [file for file in tmp if file.endswith("static.csv")]
    for csv in csvs:
        print(f"Filename: {csv}")
        df = pd.read_csv(csv)
        a2 = df.loc[df['Area'] == 2]
        a5 = df.loc[df['Area'] == 5]
        a8 = df.loc[df['Area'] == 8]
        areas = [a2,a5,a8]

        for area in areas: 
            hits = len(area[area.x != 0])
            misses = len(area[area.x == 0])
            print(f"Camera: 3, Area: {area.iloc[0]['Area']}, Accuracy: {hits/(hits+misses)*100}")

elif mode == "m":
    csvs = [file for file in tmp if file.endswith("moving.csv")]
    for csv in csvs:
        print(f"Filename: {csv}")
        df = pd.read_csv(csv)
        l1_df = df.loc[df['Line'] == 1]
        l2_df = df.loc[df['Line'] == 2]
        l3_df = df.loc[df['Line'] == 3]
        lines = [l1_df,l2_df,l3_df]
        if "simple" in csv:
            fsused = df["FastSearchUsed"].value_counts()[1]
            print(len(df))
            print(f"Fast search used in {round(fsused/len(df)*100,3)} of search cases")

        for line_df in lines:
            hits = line_df["DetectionStatus"].value_counts()[1]
            misses = line_df["DetectionStatus"].value_counts()[0]
            lineNo = line_df["Line"].iloc[0]
            print(f"For line {lineNo}, accuracy was {round(hits/(hits+misses)*100,3)} %")

elif mode == "t":
    csvs = [file for file in tmp if file.endswith("timing.csv")]
    b = False
    for csv in csvs:
        print(f"Filename: {csv}")
        df = pd.read_csv(csv)
        loop_times = []

        pd.set_option('display.float_format', lambda x: '%.5f' % x) # removes scientific notation (e.g. 1.003e+5)
        print(f"Mean:\n {df.mean(axis=0)}\nStdev:\n {df.std(axis=0)}")
        imp_time = df["ImportTime"]
        tf_time = df["TransformTime"]
        search_time = df["SearchTime"]
        timestamps = df["Timestamp"]
        bigtime = df.loc[df["SearchTime"] > 2]
        for i in range(1,len(search_time)):
            loop_times.append(timestamps.iloc[i] - timestamps.iloc[i-1])
        loop_mean_msec = mean(loop_times)
        loop_std_msec = stdev(loop_times)
        hertz_mean = 1/(loop_mean_msec/1000)
        hertz_std = 1/(loop_std_msec/1000)

        print(f"Full loop time mean: {mean(loop_times)}. Stdev: {stdev(loop_times)}\nUpdate freq mean: {hertz_mean}, std: {hertz_std}")
        plt.figure()
        plt.hist(loop_times, bins = [i for i in range(min(loop_times),max(loop_times),20)], edgecolor = "black")
        if not b:
            plt.title("Åtgången tid för hel loop, avancerad modell")
            plt.text(1500,1000,f"Mean: {mean(loop_times)}")
            plt.text(1500,900,f"stdev: {round(stdev(loop_times), 3)}")
            b = True
        else:
            plt.title("Åtgången tid för hel loop, enkel modell")
        # plt.legend(labels = [f"Mean: {mean(loop_times)}",f"stdev: {stdev(loop_times)}"])
    plt.show()

# print(df, a2, a5, a8)